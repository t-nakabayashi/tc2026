import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location(
    'configure_ntp_poll', Path(__file__).parents[1]/'tools/configure_ntp_poll.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)


def test_poll_bounds_preserve_sources_options_and_comments():
    original = ('# pool untouched\n'
                'pool ntp.ubuntu.com iburst maxsources 4 # pool\n'
                'server 192.0.2.1 minpoll 9 maxpoll 10 prefer\n'
                'server 192.0.2.2 minpoll 4 maxpoll 6\n'
                'refclock SOCK /run/chrony/um982.sock noselect\n')
    result = config.bound_polls(original)
    assert result == ('# pool untouched\n'
                      'pool ntp.ubuntu.com iburst maxsources 4 maxpoll 7 # pool\n'
                      'server 192.0.2.1 minpoll 7 maxpoll 7 prefer\n'
                      'server 192.0.2.2 minpoll 4 maxpoll 6\n'
                      'refclock SOCK /run/chrony/um982.sock noselect\n')
    assert config.bound_polls(result) == result


def test_apply_follows_fragments_backs_up_and_rolls_back_invalid_config(tmp_path, monkeypatch):
    fragments = tmp_path/'conf.d'
    fragments.mkdir()
    source = fragments/'ntp.conf'
    source.write_text('pool example.org iburst\n')
    main = tmp_path/'chrony.conf'
    main.write_text(f'confdir {fragments}\n')
    def invalid(*a, **kw):
        raise subprocess.CalledProcessError(1, 'chronyd')
    monkeypatch.setattr(config.subprocess, 'run', invalid)
    with pytest.raises(subprocess.CalledProcessError):
        config.apply(main, tmp_path/'backup')
    assert source.read_text() == 'pool example.org iburst\n'
    assert (tmp_path/'backup'/source.relative_to('/')).read_text() == source.read_text()
    monkeypatch.setattr(config.subprocess, 'run', lambda *a, **kw: None)
    config.apply(main, tmp_path/'backup2')
    assert source.read_text() == 'pool example.org iburst maxpoll 7\n'
