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
from smile.frames import Frames
from smile.nodes import Nodes
from smile.results import Results, Result
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

        results = Results(results)
        return results, anchors

    def _localize_mobile(self, mobile_node, anchors, mobile_frames):
        # Construct POLL frames filter, i.e. transmitted frames ('TX' directions) sent by mobile node
        condition = (mobile_frames.source_mac_address == mobile_node["mac_address"]) & (mobile_frames.direction == hash('TX'))
        poll_frames = mobile_frames[condition, :]

        # Construct REPONSE frames filter, i.e. transmitted frames ('RX' directions) sent to mobile node
        condition = (mobile_frames.destination_mac_address == mobile_node["mac_address"]) & (mobile_frames.direction == hash('RX'))
        response_frames = mobile_frames[condition, :]

        assert (np.unique(anchors["message_processing_time"]).shape == (1,))
        processing_delay = anchors[0, "message_processing_time"]

        # FIXME
        # sequence_numbers_triples = self._lookup_sequence_number_triples(poll_frames["sequence_number"],
        #                                                                response_frames["sequence_number"])
        sequence_numbers_triples = [(0, 1, 2), ]

        results = []  # Results.create_array(1, mac_address=mobile_node["mac_address"])

        for round_i in range(len(sequence_numbers_triples)):
            sequence_numbers = sequence_numbers_triples[round_i]

            round_poll_frames = poll_frames[np.isin(poll_frames.sequence_number, sequence_numbers), :]
            round_response_frames = response_frames[np.isin(response_frames.sequence_number, sequence_numbers), :]

            tof = round_response_frames["begin_clock_timestamp"] - round_poll_frames["begin_clock_timestamp"]
            tof -= processing_delay
            tof /= 2

            distances = np.zeros((3,))
            distances[:] = tof * self.c

            solver = self.Solver(anchors[0:3, "position_2d"], distances, self.solver_configuration)
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
            results.append(result)

        return results

    def _lookup_sequence_number_triples(self, poll_sequence_numbers, response_sequence_numbers):
        sequence_numbers = np.intersect1d(poll_sequence_numbers, response_sequence_numbers)
        triples = []

        current_triple = []
        for sequence_number in sequence_numbers:
            if len(current_triple) == 0:
                current_triple.append(sequence_number)
            elif current_triple[-1] == sequence_number - 1:
                current_triple.append(sequence_number)
            else:
                current_triple = [sequence_number]

            if len(current_triple) == 3:
                triples.append(current_triple)
                current_triple = []

        return triples
