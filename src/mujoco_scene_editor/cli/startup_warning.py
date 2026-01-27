from cachier import cachier
import sys

from rich import print


first_startup_message = """

[red][b]First startup after installation. Might take a while.[/b][/red]

"""


@cachier()
def print_first_start_info(exec_name: str = sys.executable):
    print(first_startup_message)
    return ""


print_first_start_info()
