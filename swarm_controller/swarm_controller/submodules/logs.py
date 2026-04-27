import duckdb
import json
import numpy as np
import pandas as pd

from .swarm import Agent


def init_log(db_path):
    con = duckdb.connect(db_path)
    
    con.execute("""
        CREATE TABLE IF NOT EXISTS log_data (
            timestamp DOUBLE,
            x DOUBLE,
            y DOUBLE,
            theta DOUBLE,
            v DOUBLE,
            w DOUBLE,
            v_ref DOUBLE,
            w_ref DOUBLE,
            vectors_json TEXT
        );
    """)
    
    con.close()


def write_log(db_path: str, timestamp: float, agent: Agent, peers_vecs: list, logger):
    x = agent.data['x']
    y = agent.data['y']
    theta = agent.data['theta']
    v = agent.data['v']
    w = agent.data['w']
    v_ref = agent.v_ref
    w_ref = agent.w_ref
    
    # logger.info(f'logw {db_path}')
    
    for i, vec in enumerate(peers_vecs):
        logger.info(f'{vec, vec.shape}')
        peers_vecs[i] = [vec[0, 0], vec[1, 0]]

    vectors_json = json.dumps(peers_vecs)

    con = duckdb.connect(db_path)
    con.execute("""
        INSERT INTO log_data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, [timestamp, x, y, theta, v, w, v_ref, w_ref, vectors_json])
    con.close()

    
def read_log(db_path: str):
    con = duckdb.connect(db_path)
    df = con.execute("SELECT * FROM log_data ORDER BY timestamp").df()
    con.close()

    def parse_vectors(vjson):
        try:
            vectors = json.loads(vjson)
        except:
            vectors = []
        np_vectors = []
        for v in vectors:
            if isinstance(v, (list, tuple)) and len(v) == 2:
                np_vectors.append(np.array(v).reshape(-1, 1))
        return np_vectors

    df['vectors'] = df['vectors_json'].apply(parse_vectors)
    df = df.drop(columns=['vectors_json'])
    return df
