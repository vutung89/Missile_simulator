# aes256.py
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
from Crypto.Protocol.KDF import PBKDF2
import base64
import os
import traceback
import logging
import time

SALT_SIZE = 16
KEY_SIZE = 32       # 256-bit
IV_SIZE = 16
ITERATIONS = 100_000


def derive_key(password: str, salt: bytes) -> bytes:
    """Derive a 256-bit AES key from password using PBKDF2-HMAC-SHA256."""
    start_time = time.perf_counter()
    key = PBKDF2(password.encode("utf-8"), salt, dkLen=KEY_SIZE, count=ITERATIONS)
    elapsed_time = time.perf_counter() - start_time
    print(f"  [Key Derivation] {elapsed_time*1000:.2f} ms")
    return key


def encrypt(plaintext: str, password: str) -> str:
    """
    Encrypt plaintext with AES-256-CBC.
    Returns base64-encoded string: SALT(16) + IV(16) + CIPHERTEXT
    """
    total_start = time.perf_counter()
    print("[ENCRYPTION] Starting...")
    
    plaintext_bytes = plaintext.encode("utf-8")

    salt = get_random_bytes(SALT_SIZE)
    iv   = get_random_bytes(IV_SIZE)
    key  = derive_key(password, salt)

    # Manual PKCS7 padding
    pad_len = AES.block_size - (len(plaintext_bytes) % AES.block_size)
    padded  = plaintext_bytes + bytes([pad_len] * pad_len)

    cipher_start = time.perf_counter()
    cipher     = AES.new(key, AES.MODE_CBC, iv)
    ciphertext = cipher.encrypt(padded)
    cipher_time = time.perf_counter() - cipher_start
    print(f"  [Cipher Encryption] {cipher_time*1000:.2f} ms")

    payload = salt + iv + ciphertext
    base64_start = time.perf_counter()
    result = base64.b64encode(payload).decode("utf-8")
    base64_time = time.perf_counter() - base64_start
    print(f"  [Base64 Encoding] {base64_time*1000:.2f} ms")
    
    total_time = time.perf_counter() - total_start
    print(f"[ENCRYPTION] Total time: {total_time*1000:.2f} ms")
    
    return result


def decrypt(token: str, password: str) -> str:
    """
    Decrypt a token produced by encrypt().
    Raises ValueError on wrong password or corrupted data.
    """
    total_start = time.perf_counter()
    print("[DECRYPTION] Starting...")
    
    base64_start = time.perf_counter()
    payload = base64.b64decode(token.encode("utf-8"))
    base64_time = time.perf_counter() - base64_start
    print(f"  [Base64 Decoding] {base64_time*1000:.2f} ms")

    salt       = payload[:SALT_SIZE]
    iv         = payload[SALT_SIZE:SALT_SIZE + IV_SIZE]
    ciphertext = payload[SALT_SIZE + IV_SIZE:]

    key = derive_key(password, salt)

    cipher_start = time.perf_counter()
    cipher  = AES.new(key, AES.MODE_CBC, iv)
    padded  = cipher.decrypt(ciphertext)
    cipher_time = time.perf_counter() - cipher_start
    print(f"  [Cipher Decryption] {cipher_time*1000:.2f} ms")

    # Validate and strip PKCS7 padding
    pad_len = padded[-1]
    if pad_len < 1 or pad_len > AES.block_size:
        raise ValueError("Invalid padding — wrong password or corrupted data.")
    if padded[-pad_len:] != bytes([pad_len] * pad_len):
        raise ValueError("Padding mismatch — wrong password or corrupted data.")

    total_time = time.perf_counter() - total_start
    print(f"[DECRYPTION] Total time: {total_time*1000:.2f} ms")
    
    return padded[:-pad_len].decode("utf-8")

# ── Test suite ───────────────────────────────────────────────────────────────
GREEN = "\033[92m"
RED   = "\033[91m"
RESET = "\033[0m"

passed = 0
failed = 0

def ok(name):
    global passed
    passed += 1
    print(f"{GREEN}  [PASS]{RESET} {name}")

def fail(name, reason=""):
    global failed
    failed += 1
    print(f"{RED}  [FAIL]{RESET} {name}" + (f": {reason}" if reason else ""))


# ── Test 1: roundtrip cơ bản ─────────────────────────────────────────────────
try:
    msg  = "Hello, AES-256!"
    pwd  = "super_secret_password"
    token = encrypt(msg, pwd)
    result = decrypt(token, pwd)
    assert result == msg, f"got '{result}'"
    logging.info(f"Token: {token}")
    logging.info(f"Decrypted: {result}")

    ok("Roundtrip cơ bản")
except Exception as e:
    fail("Roundtrip cơ bản", str(e))


# ── Test 2: cùng plaintext, mỗi lần sinh token khác nhau (random IV/salt) ────
try:
    msg = "same message"
    pwd = "password"
    t1  = encrypt(msg, pwd)
    t2  = encrypt(msg, pwd)
    assert t1 != t2, "hai token phải khác nhau"
    assert decrypt(t1, pwd) == msg
    assert decrypt(t2, pwd) == msg
    ok("Token khác nhau mỗi lần encrypt")
except Exception as e:
    fail("Token khác nhau mỗi lần encrypt", str(e))


# ── Test 3: sai password phải raise ValueError ────────────────────────────────
try:
    token = encrypt("secret data", "correct_password")
    try:
        decrypt(token, "wrong_password")
        fail("Sai password → phải raise ValueError")
    except ValueError:
        ok("Sai password → ValueError")
except Exception as e:
    fail("Sai password → ValueError", str(e))


# ── Test 4: chuỗi rỗng ───────────────────────────────────────────────────────
try:
    msg = ""
    pwd = "pwd"
    assert decrypt(encrypt(msg, pwd), pwd) == msg
    ok("Chuỗi rỗng")
except Exception as e:
    fail("Chuỗi rỗng", str(e))


# ── Test 5: plaintext dài (>1 block = 16 bytes) ───────────────────────────────
try:
    msg = "A" * 1000
    pwd = "longtest"
    assert decrypt(encrypt(msg, pwd), pwd) == msg
    ok("Plaintext dài (1000 ký tự)")
except Exception as e:
    fail("Plaintext dài (1000 ký tự)", str(e))


# ── Test 6: unicode / tiếng Việt ─────────────────────────────────────────────
try:
    msg = "Xin chào thế giới! 🚁 UAV guidance"
    pwd = "mật_khẩu_bí_mật"
    assert decrypt(encrypt(msg, pwd), pwd) == msg
    ok("Unicode / tiếng Việt + emoji")
except Exception as e:
    fail("Unicode / tiếng Việt + emoji", str(e))


# ── Test 7: token bị giả mạo (corrupt) ───────────────────────────────────────
try:
    
    token = encrypt("data", "pwd")
    raw   = bytearray(base64.b64decode(token))
    raw[40] ^= 0xFF          # flip bits ở phần ciphertext
    bad_token = base64.b64encode(bytes(raw)).decode()
    try:
        decrypt(bad_token, "pwd")
        fail("Token corrupt → phải raise ValueError")
    except ValueError:
        ok("Token corrupt → ValueError")
except Exception as e:
    fail("Token corrupt → ValueError", str(e))


# ── Kết quả ───────────────────────────────────────────────────────────────────
print(f"\n{'─'*40}")
print(f"Kết quả: {passed} passed, {failed} failed")
if failed == 0:
    print(f"{GREEN}Tất cả test đều pass ✓{RESET}")
else:
    print(f"{RED}{failed} test FAILED ✗{RESET}")