#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import Optional, Tuple, TYPE_CHECKING
from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np

from sinadra_configuration_parameters import PREDICTION_HORIZON, PREDICTION_TIMESTEP

if TYPE_CHECKING:
    from matplotlib.figure import Figure
    from matplotlib.axes import Axes
    from matplotlib.lines import Line2D


class RiskPlot:
    def __init__(self) -> None:
        # Configuration
        font_size = 14
        rc('font', size=font_size)  # controls default text sizes

        # Figure and Axes
        self._fig_risk: Optional["Figure"] = None
        self._ax_risk: Optional["Axes"] = None

        # X Values
        self._x: np.ndarray = np.arange(0, PREDICTION_HORIZON + PREDICTION_TIMESTEP, PREDICTION_TIMESTEP)

        # Y Values
        self._fv_risk_x_value: Optional[np.ndarray] = None
        self._fv_risk_x_value_emergency: Optional[np.ndarray] = None
        self._fv_risk_x_value_target_brake: Optional[np.ndarray] = None
        self._fv_risk_x_value_idm: Optional[np.ndarray] = None

        self._left_sv_risk_x_value: Optional[np.ndarray] = None
        self._left_sv_risk_y_value: Optional[np.ndarray] = None

        self._right_sv_risk_x_value: Optional[np.ndarray] = None
        self._right_sv_risk_y_value: Optional[np.ndarray] = None

        # Plots
        self._fv_risk_line: Optional["Line2D"] = None
        self._fv_risk_line_emergency: Optional["Line2D"] = None
        self._fv_risk_line_target_brake: Optional["Line2D"] = None
        self._fv_risk_line_idm: Optional["Line2D"] = None

        self._left_sv_risk_x_line: Optional["Line2D"] = None
        self._left_sv_risk_y_line: Optional["Line2D"] = None

        self._right_sv_risk_x_line: Optional["Line2D"] = None
        self._right_sv_risk_y_line: Optional["Line2D"] = None

        # Initialize Figure & Plots
        self._initialize_risk_plot_figure()

        self._initialize_front_vehicle_plots()
        self._initialize_left_side_vehicle_plots()
        self._initialize_right_side_vehicle_plots()

        self._ax_risk.legend(loc="best")

    def _initialize_front_vehicle_plots(self):
        # Y Values
        self._reset_front_vehicle_values()

        # Create Plots
        # default linewidth = 2.0 -> double the size of the dotted lines
        self._fv_risk_line, = self._ax_risk.plot(self._x, self._fv_risk_x_value, linewidth=4.0,
                                                 label="Weighted FV Braking Risk")
        line_style = "dotted"
        opacity = 1.0
        self._fv_risk_line_emergency, = self._ax_risk.plot(self._x, self._fv_risk_x_value_emergency, alpha=opacity,
                                                           linestyle=line_style, label="FV Emergency Brake Risk")
        self._fv_risk_line_target_brake, = self._ax_risk.plot(self._x, self._fv_risk_x_value_target_brake,
                                                              alpha=opacity, linestyle=line_style,
                                                              label="FV Target Brake Risk")
        self._fv_risk_line_idm, = self._ax_risk.plot(self._x, self._fv_risk_x_value_idm, alpha=opacity,
                                                     linestyle=line_style, label="FV IDM Risk")

    def _initialize_left_side_vehicle_plots(self):
        # Y Values
        self._reset_left_side_vehicle_values()

        # Create Plots
        self._left_sv_risk_x_line, = self._ax_risk.plot(self._x, self._left_sv_risk_x_value,
                                                        label="Left SV Cut-In Long Risk")
        self._left_sv_risk_y_line, = self._ax_risk.plot(self._x, self._left_sv_risk_y_value,
                                                        label="Left SV Cut-In Lat Risk")

    def _initialize_right_side_vehicle_plots(self):
        # Y Values
        self._reset_right_side_vehicle_values()

        # Create Plots
        self._right_sv_risk_x_line, = self._ax_risk.plot(self._x, self._right_sv_risk_x_value,
                                                         label="Right SV Cut-In Long Risk")
        self._right_sv_risk_y_line, = self._ax_risk.plot(self._x, self._right_sv_risk_y_value,
                                                         label="Right SV Cut-In Lat Risk")

    def _initialize_risk_plot_figure(self) -> None:
        self._fig_risk, self._ax_risk = plt.subplots(dpi=300)
        self._ax_risk.axis([0, PREDICTION_HORIZON, 0, 1])
        self._fig_risk.suptitle('Integral Collision Risk')
        self._ax_risk.set_xlabel("Time Horizon [s]")
        self._ax_risk.set_ylabel("Collision Probability")
        self._ax_risk.set_ylim([-0.1, 1.1])

    def update_and_draw_risk_plot(self) -> None:
        # Update dynamic risk plot with currently stored total risk
        # array
        self._fig_risk.canvas.flush_events()

        # Prediction horizon and timestep are constant over scenario
        # -> not necessary to reset x axis
        # self.risk_line.set_xdata(self.x)

        self._fv_risk_line.set_ydata(self._fv_risk_x_value)
        self._fv_risk_line_emergency.set_ydata(self._fv_risk_x_value_emergency)
        self._fv_risk_line_target_brake.set_ydata(self._fv_risk_x_value_target_brake)
        self._fv_risk_line_idm.set_ydata(self._fv_risk_x_value_idm)
        self._left_sv_risk_x_line.set_ydata(self._left_sv_risk_x_value)
        self._left_sv_risk_y_line.set_ydata(self._left_sv_risk_y_value)
        self._right_sv_risk_x_line.set_ydata(self._right_sv_risk_x_value)
        self._right_sv_risk_y_line.set_ydata(self._right_sv_risk_y_value)

        self._fig_risk.canvas.draw()

        # Assures that the plots are back to [0, ...] arrays after
        # updating the next time in case that a vehicle is not of
        # relevance anymore.
        self._reset_plot_values()

    def get_rgb_image(self) -> bytes:
        return self._fig_risk.canvas.tostring_rgb()

    def get_width_height(self) -> Tuple[int]:
        return self._fig_risk.canvas.get_width_height()

    def set_front_vehicle_cumulative_risk(self, fv_risk_x_value: np.ndarray) -> None:
        self._fv_risk_x_value = fv_risk_x_value

    def set_front_vehicle_emergency_risk(self, fv_risk_x_value_emergency: np.ndarray) -> None:
        self._fv_risk_x_value_emergency = fv_risk_x_value_emergency

    def set_front_vehicle_target_brake_risk(self, fv_risk_x_value_target_brake: np.ndarray) -> None:
        self._fv_risk_x_value_target_brake = fv_risk_x_value_target_brake

    def set_front_vehicle_idm_risk(self, fv_risk_x_value_idm: np.ndarray) -> None:
        self._fv_risk_x_value_idm = fv_risk_x_value_idm

    def set_left_side_vehicle_longitudinal_risk(self, left_sv_risk_x_value: np.ndarray) -> None:
        self._left_sv_risk_x_value = left_sv_risk_x_value

    def set_left_side_vehicle_lateral_risk(self, left_sv_risk_y_value: np.ndarray) -> None:
        self._left_sv_risk_y_value = left_sv_risk_y_value

    def set_right_side_vehicle_longitudinal_risk(self, right_sv_risk_x_value: np.ndarray) -> None:
        self._right_sv_risk_x_value = right_sv_risk_x_value

    def set_right_side_vehicle_lateral_risk(self, right_sv_risk_y_value: np.ndarray) -> None:
        self._right_sv_risk_y_value = right_sv_risk_y_value

    def save_risk_plot_on_disk(self, directory_path: str, file_name: str) -> None:
        self._fig_risk.savefig(f"{directory_path}{file_name}", dpi=300)

    def _reset_plot_values(self) -> None:
        self._reset_front_vehicle_values()
        self._reset_left_side_vehicle_values()
        self._reset_right_side_vehicle_values()

    def _reset_front_vehicle_values(self):
        self._fv_risk_x_value = np.zeros(len(self._x))
        self._fv_risk_x_value_emergency = np.zeros(len(self._x))
        self._fv_risk_x_value_target_brake = np.zeros(len(self._x))
        self._fv_risk_x_value_idm = np.zeros(len(self._x))

    def _reset_left_side_vehicle_values(self):
        self._left_sv_risk_x_value = np.zeros(len(self._x))
        self._left_sv_risk_y_value = np.zeros(len(self._x))

    def _reset_right_side_vehicle_values(self):
        self._right_sv_risk_x_value = np.zeros(len(self._x))
        self._right_sv_risk_y_value = np.zeros(len(self._x))
