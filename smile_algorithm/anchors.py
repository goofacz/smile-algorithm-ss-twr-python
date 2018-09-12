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

import smile.nodes as snodes


def _get_base_column_names():
    column_names = snodes._get_base_column_names()
    column_names["message_processing_time"] = 4

    return column_names


class Anchors(snodes.Nodes):
    _column_names = _get_base_column_names()

    def __new__(cls, input_array):
        return super(Anchors, cls).__new__(cls, input_array, Anchors._column_names)
