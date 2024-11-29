import subprocess

boards = ["base", "submarine"]
for board in boards:
    choice = input(f"Ready to upload {board}? (Enter/n): ")
    if choice == "n":
        continue

    if board == "base":
        print(f"Uploading {board}...")
        print("Copy index.html to WebPage.h...")
        with open(f"{board}/index.html", "r") as html_file:
            with open(f"{board}/WebPage.h", "w") as h_file:
                h_file.write('const char PAGE_MAIN[] PROGMEM = R"=====(')
                h_file.write(html_file.read())
                h_file.write(')=====";')

    print("Compiling...")
    compiled = subprocess.run(["arduino-cli", "compile", "--fqbn",
                               "esp32:esp32:uPesy_wroom", f"{board}/{board}.ino"])
    if (compiled.returncode):
        print("Compilation failed!")
        exit(1)

    board_list = subprocess.run(
        ["arduino-cli", "board", "list"], capture_output=True)
    board_list = board_list.stdout.decode("utf-8")
    board_list = board_list.split("\n")
    board_list = [line for line in board_list if "serial" in line]
    board_list = [line.split(" ")[0] for line in board_list]
    ESP32_board = board_list[0]
    print("Uploading to " + ESP32_board)
    subprocess.run(["arduino-cli", "upload", "-p", ESP32_board, "--fqbn",
                    "esp32:esp32:uPesy_wroom", f"{board}/{board}.ino"])
