from __future__ import annotations

import itertools
import os
import sys

import numpy as np
import pandas as pd
import sumolib as sumolib
import tqdm
from scipy.optimize import least_squares

import pickle
import SUMOUtils
from SUMOUtils import Route

sys.setrecursionlimit(100000)

INITIAL_TRAFFIC_DEMAND = 100

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools', 'route'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def create_od_pairs(I, od_pairs, initial_solution_path="init_odpairs.xml", taz_path="taz.xml"):
    edges_list = set(map(lambda i: i.split('_')[0], I))
    map_dict = {k: set() for k in edges_list}

    for i in I:
        for edge in edges_list:
            if i.startswith(edge):
                map_dict[edge].add(i)
    taz_id = 0
    with open(taz_path, 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "additional")  # noqa
        for edge in map_dict:
            outf.write("    <taz id=\"taz_{}\" edges=\"{}\"/>\n".format(edge, edge))
            taz_id += 1
        outf.write("</additional>\n")

    with open(initial_solution_path, 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "data", "datamode_file.xsd")  # noqa
        outf.write(4 * ' ' + '<interval id="%s" begin="%s" end="%s">\n' % (0, time_period_start, time_period_end))
        for od_pair in od_pairs:
            outf.write(
                8 * ' ' + '<tazRelation from="taz_%s" to="taz_%s" count="%s"/>\n' % (
                    od_pair[0], od_pair[1], str(INITIAL_TRAFFIC_DEMAND)))
        outf.write(4 * ' ' + '</interval>\n')
        outf.write('</data>\n')


def calculate_route_choice_probability(routes_list: list[Route]) -> list[Route]:
    """
    Take a list of routes and return the same, where each Route object now has a route choice probability value.

    Attenzione, data una coppia OD, il valore di probabilità deve essere diviso per la somma dei valori di probabilità
    per tutte le route relative alla coppia OD

    :param routes_list: a list of Route objects. The routes here are related to ONE UNIQUE OD PAIR
    :return: a list of Route objects
    """
    return list(map(lambda r: Route.with_probability(r, 0.5), routes_list))


def map_od_routes(net: sumolib.net,
                  routes_fname: str) -> (dict[(str, str), list[Route]], dict[(str, str), int]):
    """
    return a dict where KEY is a pair of edges id (origin/destination), value is a list of routes
    :param net: SUMO road network object
    :param od_pairs_fname: full PATH to the XML file containing the OD pairs (in edge relation format)
    :param routes_fname: full PATH to the routes definition file (SUMO format)
    :return: a DICT where KEY is a pair of edges ID (o/d), VALUE is a list of routes
    """
    od_pairs_routes_dict = {}
    route_id = 0

    # key: OD pair; value: list of set of edges of the routes for the OD pair
    route_edge_set = {}
    for the_route in sumolib.xml.parse_fast(routes_fname, 'route', ['edges']):
        route = list(map(lambda eid: net.getEdge(eid), the_route.edges.split()))
        route_edge_ids = set(map(lambda eid: net.getEdge(eid).getID(), the_route.edges.split()))
        o, d = route[0].getID(), route[-1].getID()
        if (o, d) not in route_edge_set:
            route_edge_set[(o, d)] = []
            od_pairs_routes_dict[(o, d)] = []
        if not any(map(lambda s: route_edge_ids == s, route_edge_set[(o, d)])):
            route_edge_set[(o, d)].append(route_edge_ids)
            od_pairs_routes_dict[(o, d)].append(Route(route_id, route, route[0], route[-1], 0))

    return od_pairs_routes_dict


def get_routes_by_edge(I: list[str], all_routes: list[Route]) -> dict[str, list[Route]]:
    """

    :param I: list of strings. Each string is an edge ID (sumo) containing a sensor
    :param all_routes: list of Route objects
    :return:
    """
    edges_list = set(map(lambda i: i.split('_')[0], I))
    out_ = {}  # type: dict[str, list[Route]]
    for route in all_routes:
        for i in edges_list:
            if i not in out_:
                out_[i] = []
            if route.has_edge(i):
                out_[i].append(route)
    return out_


def get_dense_od_demand_matrix(I, od_counts_dict):
    edges_list = set(map(lambda i: i.split('_')[0], I))
    theta = df_zeros(list(edges_list))
    for key, value in od_counts_dict.items():
        theta.loc[key[0], key[1]] = value
    return theta


def get_p(I, od_pairs_routes_dict):
    """

    :param I:
    :param od_pairs_routes_dict:
    :return:
    """
    edges_list = set(map(lambda i: i.split('_')[0], I))
    edges_list = list(edges_list)
    edges_list.sort()
    P = pd.DataFrame(0, index=list(edges_list), columns=list(od_pairs_routes_dict.keys()), dtype='float')
    for ij in od_pairs_routes_dict:
        i = ij[0]
        j = ij[1]
        k_ij = len(od_pairs_routes_dict[ij])
        k_j = len(list(filter(lambda r: r.has_edge(j), od_pairs_routes_dict[ij])))
        P.at[j, ij] = k_ij / k_j
        k_i = len(list(filter(lambda r: r.has_edge(i), od_pairs_routes_dict[ij])))
        P.at[i, ij] = k_ij / k_i
    return P


def df_zeros(I):
    return pd.DataFrame(0, index=I, columns=I, dtype='float')


def det_output_to_df(fname):
    intervals = list(sumolib.xml.parse_fast(fname, 'interval', ["begin", "end", "id", "nVehContrib"]))
    dd = {}
    for entry in intervals:
        begin = entry.begin
        end = entry.end
        _id = entry.id
        nveh = entry.nVehContrib
        if begin not in dd:
            dd[begin] = {}
        edge_id = _id.split('_')[0]
        if edge_id not in dd[begin]:
            dd[begin][edge_id] = 0
        dd[begin][edge_id] += eval(nveh)

    return pd.DataFrame(dd).T


if __name__ == '__main__':
    allowed_pairs = [
        ("D2", "D7"),
        ("D2", "D5"),
        ("D2", "D3"),
        ("D8", "D1"),
        ("D8", "D3"),
        ("D8", "D5"),
        ("D4", "D1"),
        ("D4", "D7"),
        ("D4", "D5"),
        ("D6", "D3"),
        ("D6", "D7"),
        ("D6", "D1")
    ]
    net_path = "data/SyntheticScenario/quickstart.net.xml"
    csv_data = "data/SyntheticScenario/veh_counts.csv"
    sensors_csv = "data/SyntheticScenario/det.add.xml"
    det_out_xml = "data/SyntheticScenario/detector.out.xml"
    vtype_add_file = "data/SyntheticScenario/vtype.add.xml"
    gt_od_file = "data/SyntheticScenario/odpairs.xml"
    I = 0  # links with sensors (edge IDs)

    os.makedirs("output", exist_ok=True)

    out_routes_path = os.path.join("output", "routes.xml")
    out_trips_path = os.path.join("output", "trips.xml")
    out_taz_path = os.path.join("output", "taz.xml")
    start_decision_vector_path = os.path.join("output", "init_odpairs.xml")
    output_decision_vector_path = os.path.join("output", "output_odpairs.xml")

    if os.path.exists(os.path.join("output", "routes.xml")):
        os.remove(out_routes_path)
        os.remove(os.path.join("output", "routes.alt.xml"))
        os.remove(os.path.join("output", "trips.xml"))

    ts = [0, 900, 1800, 3600]
    time_period_start = 0
    time_period_end = 3600

    od_gt = list(sumolib.xml.parse_fast(gt_od_file, 'tazRelation', ['from', 'to', 'count']))
    od_pairs = dict(map(lambda od: ((od.attr_from, od.to), eval(od.count)), od_gt))
    od_pairs_df = pd.DataFrame.from_dict(od_pairs, orient="index")

    # Get the traffic counts from virtual sensors and calculate covariance/correlation
    data_hour = det_output_to_df(det_out_xml)
    # set of edges ID having sensors (i consider a small subset)
    I = list(sumolib.xml.parse_fast(sensors_csv, 'inductionLoop', ['id']))
    I = list(map(lambda sensor: sensor.id, I))

    create_od_pairs(I, allowed_pairs, initial_solution_path=start_decision_vector_path, taz_path=out_taz_path)

    # Initial traffic demand
    SUMOUtils.create_trips(net_path,
                           taz_path=out_taz_path,
                           odpairs_path=start_decision_vector_path,
                           trips_path=out_trips_path,
                           out_routes_fname=out_routes_path,
                           vtype_add_file_path=vtype_add_file)

    # create a dict where KEY is an OD pair, VALUE is the list of Routes with the same OD
    # same for od_counts, but here value is the number of routes for the OD pair
    od_pairs_routes_dict = map_od_routes(sumolib.net.readNet(net_path), out_routes_path)

    # unroll the list of routes
    all_routes = list(itertools.chain(*od_pairs_routes_dict.values()))
    # Calculate R, a dict where KEY is an edge ID (in I), VALUE is a list of routes containing i (for i in I)
    R = get_routes_by_edge(I, all_routes)
    P_ = get_p(I, od_pairs_routes_dict)
    D = np.eye(len(od_pairs_routes_dict), dtype=float)
    P = P_.to_numpy()

    O_hat = np.linalg.inv(P.T @ P + 0.1 * D) @ P.T @ data_hour.sum().sort_index()
    O_hat_df = pd.DataFrame(O_hat, index=list(P_.columns))

    y_hat = P @ O_hat_df.to_numpy()

    err = ((od_pairs_df.sort_index() - O_hat_df.sort_index()) ** 2).sum().sum()

    print('input decision vector: {}'.format(od_pairs_df.sort_index()))
    print('output decision vector: {}'.format(O_hat_df.sort_index()))
    print('output decision vector Y: {}'.format(y_hat))

    with open(output_decision_vector_path, 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "data", "datamode_file.xsd")  # noqa
        outf.write(4 * ' ' + '<interval id="%s" begin="%s" end="%s">\n' % (0, time_period_start, time_period_end))
        for od, value in O_hat_df.sort_index().to_dict(orient='index').items():
            outf.write(8 * ' ' + '<edgeRelation from="%s" to="%s" count="%s"/>\n' % (od[0], od[1], int(value[0])))
        outf.write(4 * ' ' + '</interval>\n')
        outf.write('</data>\n')
