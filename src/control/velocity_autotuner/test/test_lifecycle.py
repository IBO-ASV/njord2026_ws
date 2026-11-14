"""Regression tests for asynchronous tuning completion, pause, and restore."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
import yaml

from velocity_autotuner.node import VelocityAutotuner, ControlState


def response(success=True):
    future = Future()
    future.set_result(NS(results=[NS(successful=success, reason='rejected' if not success else '')]))
    return future


@pytest.fixture
def tuner(tmp_path):
    node = VelocityAutotuner.__new__(VelocityAutotuner)
    node.output_dir = tmp_path
    node.run_id = 'test'
    node.chosen = {'ff': 2., 'p': 3., 'd': 4.}
    node.status = 'RUNNING'
    node.completed = False
    node.reserved = True
    node.param_future = None
    node.future = None
    node.stage = 'd'
    node.trial_at = 0.
    node.writer = None
    node.csv_stream = None
    node.failure = ''
    node.operation = ''
    node.state = NS(requested_mode=ControlState.MODE_MANUAL, auto_permitted=False)
    node.old = {'safety.max_thruster_thrust_n': 10., 'control.i.surge': 1.,
                'control.ff.surge': 5., 'control.p.surge': 6., 'control.d.surge': 7.}
    return node


def test_completion_applies_selected_gains_before_publishing_result(tuner):
    calls = []
    tuner._call_set = lambda values, callback: calls.append((values, callback))
    tuner._complete()
    assert tuner.status == 'FINALIZING'
    assert not list(tuner.output_dir.iterdir())
    values, callback = calls.pop()
    assert values == {'control.ff.surge': 2., 'control.p.surge': 3., 'control.d.surge': 4.}
    callback(response())
    assert tuner.status == 'COMPLETE'
    assert yaml.safe_load((tuner.output_dir / 'velocity_gains_test.yaml').read_text()) == {
        'control': {'ff': {'surge': 2.}, 'p': {'surge': 3.}, 'd': {'surge': 4.}}}


def test_rejected_final_gains_do_not_publish_success(tuner):
    tuner._completed(response(False))
    assert tuner.status == 'PAUSED'
    assert not tuner.completed
    assert not list(tuner.output_dir.iterdir())


@pytest.mark.parametrize('transport_error', [False, True])
def test_failed_restore_keeps_session_reserved_for_retry(tuner, transport_error):
    future = response(False)
    if transport_error:
        future = Future()
        future.set_exception(RuntimeError('connection lost'))
    tuner._restored(future)
    assert tuner.status == 'PAUSED'
    assert tuner.reserved
    assert 'Restore failed' in tuner.failure


def test_successful_restore_releases_session(tuner):
    tuner._restored(response())
    assert tuner.status == 'IDLE'
    assert not tuner.reserved


def test_stop_waits_for_inflight_parameter_write(tuner):
    tuner.param_future = Future()
    result = tuner._stop(None, NS())
    assert not result.success
    assert tuner.reserved


def test_restore_retry_retains_selected_gains_after_completion(tuner):
    tuner.completed = True
    tuner.status = 'PAUSED'
    calls = []
    tuner._call_set = lambda values, cb: calls.append(values)
    assert tuner._stop(None, NS()).success
    assert calls == [{'safety.max_thruster_thrust_n': 10., 'control.i.surge': 1.}]


def test_candidate_callback_cannot_restart_paused_trial(tuner):
    tuner.status = 'PAUSED'
    tuner._candidate_applied(response())
    assert tuner.trial_at == 0.
    assert tuner.status == 'PAUSED'


def test_preparation_callback_does_not_cancel_pause(tuner):
    tuner.status = 'PAUSED'
    stages = []
    tuner._next_stage = stages.append
    tuner._prepared(response())
    assert tuner.status == 'PAUSED'
    assert stages == ['ff']
    tuner._close_csv()


def test_fault_at_trial_deadline_does_not_record_or_advance(tuner):
    import time
    tuner.trial_at = time.monotonic() - 100
    tuner.pre_stop = 3.
    tuner.settle = 25.
    tuner.window = 8.
    tuner.step_duration = 18.
    tuner.odom_at = tuner.force_at = 0.
    tuner.candidate = 4.
    published = []
    tuner.active_pub = NS(publish=lambda msg: None)
    tuner.status_pub = NS(publish=lambda msg: None)
    tuner.cmd_pub = NS(publish=published.append)
    tuner._finish_trial = lambda: pytest.fail('faulted trial must not advance')
    tuner._tick()
    assert tuner.status == 'PAUSED'
    assert published[-1].linear.x == 0.


def test_rejected_gain_cannot_be_resumed_as_a_valid_trial(tuner):
    tuner._candidate_applied(response(False))
    tuner.state = NS(requested_mode=ControlState.MODE_AUTO, auto_permitted=True)
    assert not tuner._resume(None, NS()).success
    assert tuner.trial_at == 0.


def test_ff_uses_entire_configured_steady_window(tuner):
    tuner.stage = 'ff'
    tuner.pre_stop = 3.
    tuner.settle = 25.
    tuner.target = 1.
    tuner.samples = [(25., 1.), (26., 1.), (27., 1.)]
    recorded = []
    tuner.search = NS(record=lambda *args: recorded.append(args))
    tuner.candidate = 2.
    tuner._write_row = lambda *args: None
    tuner._next_candidate = lambda: None
    tuner._finish_trial()
    assert recorded == [(2., 0., 0.)]
