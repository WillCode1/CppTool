import base64
import re
import os

desktop_path = os.path.join(os.path.expanduser('~'), "Desktop\\")


def file_to_base64(file_name):
    file = open(desktop_path + file_name, 'rb')
    base64_bytes = base64.b64encode(file.read())
    file.close()
    # print(base64_bytes)
    print(len(base64_bytes))
    base64_str = bytes.decode(base64_bytes)
    return base64_str


def base64_to_file(base64_code, decrypt_file):
    base64_data = base64.b64decode(base64_code)
    file = open(desktop_path + decrypt_file, 'wb')
    file.write(base64_data)
    file.close()
    print("Decrypt Successfully!")


def print_long_str_to_file(base64_code):
    base64_list = re.findall(r'.{100000}', base64_code)

    for index in range(len(base64_list)):
        base64_list[index] += '\n'

    base64_list.append(base64_code[len(base64_list) * 100000:] + '\n')

    content = ''.join(base64_list)
    write_file = open(desktop_path + "tmp.tmp", 'w')
    write_file.write(str(content))
    write_file.close()
    print("Encrypt Successfully!")


def read_base64_from_file():
    base64_file = open(desktop_path + "tmp.tmp", 'r')
    all_lines = base64_file.readlines()
    base64_str = ""
    for string in all_lines:
        base64_str += string

    base64_str = base64_str.replace('\n', '')
    base64_bytes = str.encode(base64_str)
    return base64_bytes


if __name__ == "__main__":
    encrypt_file = '1.rar'
    decrypt_file = 'bookmarks.rar'

    # 加密
    # .rar -> base64
    # print_long_str_to_file(file_to_base64(encrypt_file))

    # 解密
    base64_to_file(read_base64_from_file(), decrypt_file)
