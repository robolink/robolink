import os

from rosdistro import get_index

FILES_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'files'))


def test_get_index():
    url = 'file://' + FILES_DIR + '/index.yaml'
    i = get_index(url)
    assert len(i.distributions.keys()) == 1
    assert 'foo' in i.distributions.keys()
