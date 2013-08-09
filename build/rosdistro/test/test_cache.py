import os

from rosdistro import get_index, get_release_cache

FILES_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'files'))


def test_get_release_cache():
    url = 'file://' + FILES_DIR + '/index.yaml'
    i = get_index(url)
    get_release_cache(i, 'foo')
