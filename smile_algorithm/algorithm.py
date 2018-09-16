#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http:#www.gnu.org/licenses/.
#

import importlib
import os.path

import numpy as np
import scipy.constants as scc

import smile.area as sarea
import smile.algorithm as salgorithm
import smile.results as sresults
from smile.frames import Frames
from smile.nodes import Nodes
from smile.result import Result
from smile_algorithm.anchors import Anchors


class Algorithm(salgorithm.Algorithm):
    def __init__(self, configuration):
        self._configuration = configuration
        self.solver_configuration = self._configuration['algorithms']['tof']['solver']

        solver_module = importlib.import_module(self.solver_configuration['module'])
        self.Solver = getattr(solver_module, self.solver_configuration['class'])

        area_configuration = self._configuration['area']
        area_file_path = area_configuration['file']
        self.area = sarea.Area(area_file_path)

        self.c = scc.value('speed of light in vacuum')
        self.c *= 1e-12  # m/s -> m/ps

    def run_offline(self, directory_path):
        anchors = Anchors(os.path.join(directory_path, 'ss_twr_anchors.csv'))
        mobiles = Nodes(os.path.join(directory_path, 'ss_twr_mobiles.csv'))
        mobile_frames = Frames(os.path.join(directory_path, 'ss_twr_mobile_frames.csv'))

        results = []
        for mobile_node in mobiles:
            mobile_results = self._localize_mobile(mobile_node, anchors, mobile_frames)
            results.extend(mobile_results)

        results = sresults.create_results(results)
        return results, anchors

    @staticmethod
    def _iterate_sequence_number_triples(poll_sequence_numbers, response_sequence_numbers):
        for i in range(0, len(poll_sequence_numbers) - 2):
            sequence_numbers = poll_sequence_numbers[[*range(i, i + 3)]]
            if np.all(np.in1d(sequence_numbers, response_sequence_numbers)):
                yield sequence_numbers

    def _localize_mobile(self, mobile_node, anchors, mobile_frames):
        # Construct POLL frames filter, i.e. transmitted frames ('TX' directions) sent by mobile node
        condition = (mobile_frames.source_mac_address == mobile_node.mac_address) & (mobile_frames.direction == hash('TX'))
        poll_frames = mobile_frames[condition, :]

        # Construct REPONSE frames filter, i.e. transmitted frames ('RX' directions) sent to mobile node
        condition = (mobile_frames.destination_mac_address == mobile_node.mac_address) & (mobile_frames.direction == hash('RX'))
        response_frames = mobile_frames[condition, :]

        # Here we will store all results
        results = []

        for sequence_numbers in Algorithm._iterate_sequence_number_triples(poll_frames.sequence_number,
                                                                           response_frames.sequence_number):
            round_poll_frames = poll_frames[np.isin(poll_frames.sequence_number, sequence_numbers), :]
            round_response_frames = response_frames[np.isin(response_frames.sequence_number, sequence_numbers), :]
            round_anchors = anchors[anchors.mac_address.find_order(round_response_frames.source_mac_address)]

            tof = round_response_frames.begin_clock_timestamp - round_poll_frames.begin_clock_timestamp
            tof -= round_anchors.message_processing_time
            tof /= 2

            distances = np.zeros((3,))
            distances[:] = tof * self.c

            solver = self.Solver(round_anchors[:, "position_2d"], distances, self.solver_configuration)
            position = solver.localize()[0]  # FIXME

            result = Result()
            result.mac_address = mobile_node["mac_address"]
            result.position_dimensions = 2
            result.position_x = position[0]
            result.position_y = position[1]
            result.position_z = 0
            result.begin_true_position_x = round_poll_frames[0, "begin_true_position_x"]
            result.begin_true_position_y = round_poll_frames[0, "begin_true_position_y"]
            result.begin_true_position_z = 0
            result.end_true_position_x = round_response_frames[2, "end_true_position_x"]
            result.end_true_position_y = round_response_frames[2, "end_true_position_y"]
            result.end_true_position_z = 0
            result.reference_position_x = result.end_true_position_x
            result.reference_position_y = result.end_true_position_y
            result.reference_position_z = 0
            results.append(result)

        return results
