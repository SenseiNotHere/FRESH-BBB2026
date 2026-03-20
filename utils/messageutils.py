def log(subsystem: str, msg: str) -> None:
    print(f"[{subsystem}] {msg}")

def print_banner(text: str) -> None:
    print("")
    print("         FRC TEAM 1811 - FRESH         ")
    print("            BOBBY THE B BOX            ")
    print(f"{text.center(39)}")
    print("")