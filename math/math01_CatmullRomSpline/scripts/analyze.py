"""
根据保存的json文件分析CR曲线
1. 从文件中读取特定(fusion element.id)的instance, 绘图并进行比较
"""

import argparse, logging, coloredlogs, datetime
from typing import List, Optional
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib


class Analyzer:
    def __init__(self, data_folder: Path) -> None:
        self.data_folder: Path = data_folder.resolve()  # data folder

        self.__raw_points: np.ndarray = None  # raw points
        self.__raw_ctrl_points: np.ndarray = None  # raw control points
        # before optimization
        self.__opt_points_before_opt: np.ndarray = None  # optimized points, sampled from optimized splines
        self.__estimated_points_before_opt: np.ndarray = (
            None  # estimated points, corresponding to raw point, [0,0] for not used
        )
        self.__opt_ctrl_points_before_opt: np.ndarray = None  # optimized control points
        # after optimization
        self.__opt_points: np.ndarray = None  # optimized points, sampled from optimized splines
        self.__estimated_points: np.ndarray = None  # estimated points, corresponding to raw point, [0,0] for not used
        self.__opt_ctrl_points: np.ndarray = None  # optimized control points

        # plot settings
        self.__figsize = (7, 9)
        # self.__colors = ['b', 'r', 'k', 'm']
        self.__colors = [
            (0.0, 0.0, 1.0, 1.0),
            (1.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, 1.0),
            (1.0, 0.0, 1.0, 1.0),
        ]  # b,r,k,m
        colormap = matplotlib.colormaps['viridis']
        self.__colors.extend([colormap(i / 30) for i in range(30)])

    def run(self) -> None:
        # load points
        self.__raw_points = self.__load_point(self.data_folder / 'RawPoints.csv')
        # self.__raw_ctrl_points = self.__load_point(self.data_folder / 'RawCtrlPoints.csv')
        self.__opt_points_before_opt = self.__load_point(self.data_folder / 'OptPointsBeforeOpt.csv')
        self.__estimated_points_before_opt = self.__load_point(self.data_folder / 'EstimatedPointsBeforeOpt.csv')
        self.__opt_ctrl_points_before_opt = self.__load_point(self.data_folder / 'OptCtrlPointsBeforeOpt.csv')
        self.__opt_points = self.__load_point(self.data_folder / 'OptPoints.csv')
        self.__estimated_points = self.__load_point(self.data_folder / 'EstimatedPoints.csv')
        self.__opt_ctrl_points = self.__load_point(self.data_folder / 'OptCtrlPoints.csv')

        # plot
        self.__plot()

        plt.show(block=True)

    def __load_point(self, file: Path) -> np.ndarray:
        if not file.exists():
            logging.error(f"input file don't exist: {file}")
            return None

        data = np.loadtxt(file, comments='#', delimiter=',')
        return None if len(data) == 0 else data

    def __plot(self) -> None:
        # plot
        fig = plt.figure('Catmull-Rom Spline Analyzer', figsize=self.__figsize)
        ax = fig.subplots(1, 1)
        ax.plot(self.__raw_points[:, 0], self.__raw_points[:, 1], 'bo-', markersize=5, label='Raw Points')
        if self.__raw_ctrl_points is not None:
            ax.plot(
                self.__raw_ctrl_points[:, 0], self.__raw_ctrl_points[:, 1], 'b*', markersize=10, label='Raw Ctrl Points'
            )

        # before opt
        if self.__opt_points_before_opt is not None:
            ax.plot(
                self.__opt_points_before_opt[:, 0],
                self.__opt_points_before_opt[:, 1],
                'c.-',
                markersize=7,
                label='Optimized Points before Opt',
            )
        if self.__opt_ctrl_points_before_opt is not None:
            ax.plot(
                self.__opt_ctrl_points_before_opt[:, 0],
                self.__opt_ctrl_points_before_opt[:, 1],
                'c*',
                markersize=12,
                label='Opt Ctrl Points before Opt',
            )
        if self.__estimated_points_before_opt is not None:
            estimated_points_before_opt = self.__estimated_points_before_opt[
                np.linalg.norm(self.__estimated_points_before_opt, axis=1) != 0, :
            ]
            ax.plot(
                estimated_points_before_opt[:, 0],
                estimated_points_before_opt[:, 1],
                'co',
                markersize=5,
                label='Estimated Points before Opt',
            )
            # line
            for p0, p1 in zip(self.__raw_points, self.__estimated_points_before_opt):
                if np.linalg.norm(p1) != 0:
                    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'c--')

        # after opt
        if self.__opt_points is not None:
            ax.plot(self.__opt_points[:, 0], self.__opt_points[:, 1], 'r-', markersize=7, label='Optimized Points')
        if self.__opt_ctrl_points is not None:
            ax.plot(
                self.__opt_ctrl_points[:, 0], self.__opt_ctrl_points[:, 1], 'r*', markersize=12, label='Opt Ctrl Points'
            )
        if self.__estimated_points is not None:
            # filter zero data
            estimated_points = self.__estimated_points[np.linalg.norm(self.__estimated_points, axis=1) != 0, :]
            ax.plot(estimated_points[:, 0], estimated_points[:, 1], 'ro', markersize=5, label='Estimated Points')
            # line
            for p0, p1 in zip(self.__raw_points, self.__estimated_points):
                if np.linalg.norm(p1) != 0:
                    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'r--')

        # title and legend
        ax.set_title('Catmull-Rom Spline Analyzer')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.grid()
        ax.legend()
        ax.set_aspect('equal')
        fig.tight_layout()


if __name__ == '__main__':
    # argument parser
    parser = argparse.ArgumentParser(
        description='Catmull-Rom Spline Analyzer',
        formatter_class=lambda prog: argparse.HelpFormatter(prog, max_help_position=40, width=120),
    )
    parser.add_argument('--path', default='./temp/CatmullRomSpline', help='data folder(default: %(default)s)')
    args = parser.parse_args()
    print(args)

    # config logging
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s %(levelname)s %(filename)s:%(lineno)d] %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(
                f"/tmp/{Path(__file__).stem}.{datetime.datetime.now().strftime('%Y%m%d-%H%M%S.%f')}.log"
            ),
        ],
    )
    coloredlogs.install(fmt="[%(asctime)s %(levelname)s %(filename)s:%(lineno)d] %(message)s")

    analyzer = Analyzer(Path(args.path))
    analyzer.run()
