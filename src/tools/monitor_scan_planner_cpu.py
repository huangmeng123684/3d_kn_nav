#!/usr/bin/env python3
"""Record scan_planner_node CPU usage and generate a PNG report."""

import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time

try:
    import psutil
except ImportError:
    print('Missing dependency: sudo apt install python3-psutil', file=sys.stderr)
    raise SystemExit(2)


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / 'log' / 'cpu'


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Wait for scan_planner_node, record its CPU and memory usage, '
            'then generate CSV and PNG reports.'
        )
    )
    parser.add_argument(
        '--pid', type=int,
        help='Monitor this PID instead of searching by executable name.',
    )
    parser.add_argument(
        '--process-name', default='scan_planner_node',
        help='Executable name to wait for (default: scan_planner_node).',
    )
    parser.add_argument(
        '--interval', type=float, default=1.0,
        help='Sampling interval in seconds (default: 1.0).',
    )
    parser.add_argument(
        '--duration', type=float, default=0.0,
        help='Stop after this many seconds; 0 means until process exit or Ctrl+C.',
    )
    parser.add_argument(
        '--print-every', type=float, default=5.0,
        help='Console status interval in seconds (default: 5.0).',
    )
    parser.add_argument(
        '--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR,
        help=f'Report directory (default: {DEFAULT_OUTPUT_DIR}).',
    )
    args = parser.parse_args()

    if args.interval <= 0.0:
        parser.error('--interval must be positive')
    if args.duration < 0.0:
        parser.error('--duration must be non-negative')
    if args.print_every <= 0.0:
        parser.error('--print-every must be positive')
    return args


def process_executable_name(process):
    try:
        executable = process.exe()
        if executable:
            return Path(executable).name
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        pass

    try:
        command = process.cmdline()
        if command:
            return Path(command[0]).name
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        pass
    return ''


def find_process(name):
    matches = []
    own_pid = os.getpid()
    for process in psutil.process_iter(['pid', 'create_time']):
        if process.pid == own_pid:
            continue
        if process_executable_name(process) == name:
            matches.append(process)

    if not matches:
        return None
    return max(matches, key=lambda process: process.info.get('create_time') or 0.0)


def wait_for_process(args):
    if args.pid is not None:
        try:
            return psutil.Process(args.pid)
        except psutil.NoSuchProcess:
            raise SystemExit(f'PID {args.pid} does not exist')

    print(f'Waiting for executable: {args.process_name}', flush=True)
    last_message = 0.0
    while True:
        process = find_process(args.process_name)
        if process is not None:
            return process

        now = time.monotonic()
        if now - last_message >= 5.0:
            print('  still waiting...', flush=True)
            last_message = now
        try:
            time.sleep(min(args.interval, 1.0))
        except KeyboardInterrupt:
            raise SystemExit('Stopped while waiting for scan_planner_node')


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = fraction * (len(ordered) - 1)
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def save_plot(records, png_path, process_name, pid):
    if not records:
        print('No samples collected; PNG was not generated.', file=sys.stderr)
        return

    cache_dir = Path(tempfile.gettempdir()) / 'scan_cpu_matplotlib'
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('MPLBACKEND', 'Agg')
    os.environ.setdefault('MPLCONFIGDIR', str(cache_dir))

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            'CSV saved, but matplotlib is missing: sudo apt install python3-matplotlib',
            file=sys.stderr,
        )
        return

    elapsed = [record['elapsed_s'] for record in records]
    process_cpu = [record['process_cpu_percent'] for record in records]
    system_cpu = [record['system_cpu_percent'] for record in records]
    rss_mb = [record['rss_mb'] for record in records]

    average = statistics.fmean(process_cpu)
    peak = max(process_cpu)
    p95 = percentile(process_cpu, 0.95)

    figure, (cpu_axis, memory_axis) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, constrained_layout=True,
    )
    cpu_axis.plot(elapsed, process_cpu, color='tab:red', label='scan_planner CPU')
    cpu_axis.plot(
        elapsed, system_cpu, color='tab:blue', alpha=0.55,
        label='system CPU',
    )
    cpu_axis.axhline(average, color='tab:red', linestyle='--', alpha=0.6)
    cpu_axis.set_ylabel('CPU (%)')
    cpu_axis.set_ylim(bottom=0.0)
    cpu_axis.grid(True, alpha=0.3)
    cpu_axis.legend(loc='upper right')
    cpu_axis.set_title(
        f'{process_name} (PID {pid})  avg={average:.1f}%  '
        f'p95={p95:.1f}%  max={peak:.1f}%'
    )

    memory_axis.plot(elapsed, rss_mb, color='tab:green', label='RSS memory')
    memory_axis.set_xlabel('Elapsed time (s)')
    memory_axis.set_ylabel('RSS (MB)')
    memory_axis.set_ylim(bottom=0.0)
    memory_axis.grid(True, alpha=0.3)
    memory_axis.legend(loc='upper right')

    figure.savefig(png_path, dpi=160)
    plt.close(figure)


def monitor(args):
    process = wait_for_process(args)
    process_name = process_executable_name(process) or process.name()
    pid = process.pid

    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    file_stem = f'scan_planner_cpu_{run_id}'
    csv_path = args.output_dir / f'{file_stem}.csv'
    png_path = args.output_dir / f'{file_stem}.png'

    print(f'Monitoring {process_name}, PID={pid}, interval={args.interval:.2f}s')
    print(f'CSV: {csv_path}')
    print('Press Ctrl+C to stop and generate the plot.', flush=True)

    records = []
    start_time = time.monotonic()
    next_status_time = start_time

    process.cpu_percent(interval=None)
    psutil.cpu_percent(interval=None)

    fieldnames = [
        'timestamp', 'elapsed_s', 'pid', 'process_cpu_percent',
        'system_cpu_percent', 'rss_mb', 'memory_percent', 'thread_count',
    ]
    reason = 'process exited'

    with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while True:
                time.sleep(args.interval)
                elapsed = time.monotonic() - start_time
                if args.duration > 0.0 and elapsed > args.duration:
                    reason = 'duration reached'
                    break

                try:
                    with process.oneshot():
                        process_cpu = process.cpu_percent(interval=None)
                        memory = process.memory_info()
                        memory_percent = process.memory_percent()
                        thread_count = process.num_threads()
                except (psutil.NoSuchProcess, psutil.ZombieProcess):
                    break

                record = {
                    'timestamp': datetime.now().astimezone().isoformat(
                        timespec='milliseconds'
                    ),
                    'elapsed_s': round(elapsed, 3),
                    'pid': pid,
                    'process_cpu_percent': round(process_cpu, 2),
                    'system_cpu_percent': round(psutil.cpu_percent(interval=None), 2),
                    'rss_mb': round(memory.rss / (1024.0 * 1024.0), 2),
                    'memory_percent': round(memory_percent, 3),
                    'thread_count': thread_count,
                }
                writer.writerow(record)
                csv_file.flush()
                records.append(record)

                now = time.monotonic()
                if now >= next_status_time:
                    print(
                        f"[{record['elapsed_s']:8.1f}s] "
                        f"CPU={record['process_cpu_percent']:6.1f}%  "
                        f"RSS={record['rss_mb']:7.1f} MB  "
                        f"threads={thread_count}",
                        flush=True,
                    )
                    next_status_time = now + args.print_every
        except KeyboardInterrupt:
            reason = 'Ctrl+C'

    save_plot(records, png_path, process_name, pid)

    if records:
        cpu_values = [record['process_cpu_percent'] for record in records]
        print(
            f'Stopped: {reason}; samples={len(records)}, '
            f'avg={statistics.fmean(cpu_values):.1f}%, '
            f'p95={percentile(cpu_values, 0.95):.1f}%, '
            f'max={max(cpu_values):.1f}%'
        )
        print(f'PNG: {png_path}')
    print(f'CSV: {csv_path}')


def main():
    monitor(parse_args())


if __name__ == '__main__':
    main()
