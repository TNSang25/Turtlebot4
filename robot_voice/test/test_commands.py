# SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from robot_semantic.maps import RegionSet
from robot_voice.commands import Command, CommandParser, UnknownRegion, make_route, normalize


@pytest.fixture
def parser():
    src = Path(__file__).resolve().parents[2]
    return CommandParser(RegionSet(src/'robot_semantic/config/regions.yaml').regions)


@pytest.mark.parametrize('text,goal', [
    ('Đi đến khu vực A.', 'A'), ('DI DEN KHU VUC A', 'A'),
    ('Đi đến phòng thí nghiệm.', 'C'), ('Di chuyển đến hành lang', 'B'),
    ('Robot hãy đi tới phòng học!', 'A'), ('Xin đi đến khu vực bê nhé', 'B'),
    ('Đi đến khu vực xê', 'C'), ('đi đến phòng lab', 'C'),
])
def test_vietnamese_navigation_commands(parser, text, goal):
    assert parser.parse(text) == Command('navigate', goal)


def test_from_a_to_b_visits_source_first_unless_robot_is_already_in_a(parser):
    command = parser.parse('Di chuyển từ khu vực A đến khu vực B.')
    assert make_route(command, 'C') == ['A', 'B']
    assert make_route(command, None) == ['A', 'B']
    assert make_route(command, 'A') == ['B']
    assert make_route(parser.parse('Đi từ A đến A'), 'C') == ['A']


@pytest.mark.parametrize('text', ['Đi đến khu vực D', 'Đi đến phòng bếp', 'Đi từ khu vực Z đến B',
                                  'Đi đến A và B', 'Đi đến phòng học thứ hai'])
def test_unknown_or_ambiguous_regions_never_become_a_nearby_known_goal(parser, text):
    with pytest.raises(UnknownRegion):
        parser.parse(text)


@pytest.mark.parametrize('text', ['Dừng lại', 'Hủy', 'Robot dừng lại', 'hãy hủy điều hướng'])
def test_voice_stop(parser, text):
    command = parser.parse(text)
    assert command.kind == 'stop'
    assert make_route(command) == []


@pytest.mark.parametrize('text', ['', 'phòng học', 'tôi đang nói về đi đến khu vực A', 'đi đến', 'A B C'])
def test_partial_or_casual_utterances_do_not_trigger_navigation(parser, text):
    with pytest.raises(ValueError):
        parser.parse(text)


def test_alias_collisions_are_configuration_errors():
    with pytest.raises(ValueError, match='Ambiguous'):
        CommandParser([NS(id='A', name='Phòng 1', aliases=['lab']), NS(id='B', name='Phòng 2', aliases=['LAB'])])
    assert normalize('ĐI ĐẾN Phòng thí nghiệm!') == 'di den phong thi nghiem'
