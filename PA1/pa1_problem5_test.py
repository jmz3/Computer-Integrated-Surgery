import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
from cispa.Registration import regist_matched_points
import cispa.DataProcess as DP
from cispa.PivotCalibration import calib_pivot_points
import numpy as np
import logging
import click
from rich.logging import RichHandler

logging.basicConfig(
        level="DEBUG",
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )

log = logging.getLogger(__name__)

@click.command()
@click.option("--data-dir", "-d", default="PA1/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA1/output", help="Output directory")
@click.option("--name", "-n", default="pa1-debug-a", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()

    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)
    if not output_dir.exists():
        output_dir.mkdir()
    
    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])

    G0 = pivot_data[0:6,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []

    for i in range(NFrames):
        G = pivot_data[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    p_t, p_pivot = calib_pivot_points(F_G)

    log.info(f"Pt = \n{p_t} \nPpivot = \n{p_pivot}")

if __name__ == "__main__":
    main()






