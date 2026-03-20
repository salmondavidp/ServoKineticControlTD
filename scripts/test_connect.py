import multiprocessing, sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'python_soem-main', 'app'))

def child():
    import pysoem
    adapter = r'\Device\NPF_{AE88A581-6D1F-45F0-9B72-75EF98EB52DD}'
    m = pysoem.Master()
    m.open(adapter)
    n = m.config_init()
    print(f'Child process: slaves found = {n}')
    m.close()

if __name__ == '__main__':
    multiprocessing.freeze_support()
    p = multiprocessing.Process(target=child)
    p.start()
    p.join(timeout=10)
    print(f'Exit code: {p.exitcode}')
