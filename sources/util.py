import logging
import os

def get_logger(name: str):
    log = logging.getLogger(name)
    log.setLevel(logging.INFO)

    # define file handler and set formatter
    log_dir = "../../log"
    #log_dir = os.environ.get('MODSPLIT_HOME', '')+"/log"

    if (not os.path.isdir(log_dir)):
        os.mkdir(log_dir,mode=0o777) 

    file_handler = logging.FileHandler(log_dir+ f'/{name}.log')
    formatter = logging.Formatter('%(asctime)s : %(message)s')
    file_handler.setFormatter(formatter)
    log.addHandler(file_handler)
    return log
