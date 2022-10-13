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
    if not output_dir.exists():
        output_dir.mkdir()

    cal_body_path = data_dir / f"{name}-calbody.txt"
    cal_body,calbody_info = DP.load_txt_data(cal_body_path)

    cal_read_path = data_dir / f"{name}-calreadings.txt"
    cal_read,cal_read_info = DP.load_txt_data(cal_read_path)

    d = cal_body[0:8,:].T
    D = cal_read[0:8,:].T
    F_d = regist_matched_points(d,D)
    F_dinv = np.linalg.inv(F_d)

    opt_pivot_path = data_dir / f"{name}-optpivot.txt"
    opt_pivot_data,opt_pivot_info = DP.load_txt_data(opt_pivot_path)

    NFrames = int(opt_pivot_info[2])
    NH = int(opt_pivot_info[1])
    ND = int(opt_pivot_info[0])

    D0 = F_dinv @ DP.homovec(opt_pivot_data[ND:ND+NH,:].T)
    D0 = DP.dehomovec(D0)
    d,d_bar = DP.centralize(D0)
    F = []

    for i in range(NFrames):

        D = F_dinv @ DP.homovec(opt_pivot_data[(ND+NH)*i+ND:(NH+ND)*(i+1),:].T)
        D = DP.dehomovec(D)
        F.append(regist_matched_points(d,D))

    # print(F[1])

    p_t, p_pivot = calib_pivot_points(F)

    log.info(f"Pt = \n{p_t} \nPpivot = \n{p_pivot}")

if __name__ == "__main__":
    main()






