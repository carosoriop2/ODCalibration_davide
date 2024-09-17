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

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools', 'route'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

iic = 0


def edge_utility(e: sumolib.net.edge) -> float:
    return e.getLength() / e.getSpeed()


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
                    od_pair[0], od_pair[1], str(10)))
        outf.write(4 * ' ' + '</interval>\n')
        outf.write('</data>\n')


def calculate_route_choice_probability(routes_list: list[Route]) -> list[Route]:
    """
    Take a list of routes and return the same, where each Route object now has a route choice probability value.

    Attenzione, data una coppia OD, il valore di probabilità deve essere diviso per la somma dei valori di probabilità
    per tutte le route relative alla coppia OD

    :param routes_list: a list of Route objects
    :return: a list of Route objects
    """
    return list(map(lambda r: Route.with_probability(r, 1 / len(routes_list)), routes_list))

    """ui_max = 0
    Uim = map(lambda route: np.sum(list(map(lambda e: edge_utility(e), route.edges))) / len(route.edges), routes_list)
    Ui_avg = np.average(list(Uim))
    route_prob = dict(map(lambda route: (route.id, (route, Ui_avg)), routes_list))
    for route in routes_list:
        Ui = np.sum(list(map(lambda e: edge_utility(e), route.edges))) / len(route.edges)
        if Ui > ui_max:
            ui_max = Ui
    norm_ei = {k: (v[0], np.e ** (v[1] - ui_max)) for k, v in route_prob.items()}
    return list(map(lambda v: Route.with_probability(v[0], v[1]), norm_ei.values()))"""


def get_route_pair(k):
    return (k[0].split('taz_')[1], k[1].split('taz_')[1])


def map_od_routes(net: sumolib.net,
                  od_pairs_fname: str,
                  routes_fname: str) -> (dict[(str, str), list[Route]], dict[(str, str), int]):
    """
    return a dict where KEY is a pair of edges id (origin/destination), value is a list of routes
    :param net: SUMO road network object
    :param od_pairs_fname: full PATH to the XML file containing the OD pairs (in edge relation format)
    :param routes_fname: full PATH to the routes definition file (SUMO format)
    :return: a DICT where KEY is a pair of edges ID (o/d), VALUE is a list of routes
    """
    od_pairs_routes_dict = {}
    od_counts = {}
    route_id = 0
    od = list(sumolib.xml.parse_fast(od_pairs_fname, 'tazRelation', ['from', 'to', 'count']))
    mapped_od = list(map(lambda od_pair: (od_pair.attr_from, od_pair.to), od))

    for the_route in sumolib.xml.parse_fast(routes_fname, 'route', ['edges']):
        route = list(map(lambda eid: net.getEdge(eid), the_route.edges.split()))
        o, d = route[0].getID(), route[-1].getID()
        key = ("taz_" + o, "taz_" + d)
        if key in mapped_od:
            if (o, d) not in od_pairs_routes_dict:
                od_pairs_routes_dict[(o, d)] = []
                od_counts[(o, d)] = 0
            od_pairs_routes_dict[(o, d)].append(Route(route_id, route, route[0], route[-1], 0))
            od_counts[(o, d)] += 1
        route_id += 1

    od_pairs_routes_dict = {k: v for k, v in od_pairs_routes_dict.items() if v}
    od_counts = {k: v for k, v in od_counts.items() if k in od_pairs_routes_dict}
    return od_pairs_routes_dict, od_counts


def get_lambda_ij(theta, I: list[str], R: dict[str, list[Route]]):
    """

    :param Pr_i: a vector where the i-th position is the probability of the routes containing the i-th edge in I
    :param theta: a matrix where
    :param theta:
    :return:
    """
    edges_list = set(map(lambda i: i.split('_')[0], I))
    all_pairs_i = list(itertools.permutations(edges_list, 2))
    lambda_ij_mat = df_zeros(list(edges_list))
    for i, j in all_pairs_i:
        lambda_ij_mat.loc[i, j] = 0

    for i, j in all_pairs_i:
        for r_i in R[i]:
            for r_j in R[j]:
                ri_edges = set(map(lambda e: e.getID(), r_i.edges))
                rj_edges = set(map(lambda e: e.getID(), r_j.edges))
                if ri_edges == rj_edges:
                    lambda_ij_mat.loc[i, j] = r_i.choice_probability * (1 - r_i.choice_probability) * theta.loc[
                        r_i.origin.getID(), r_i.dest.getID()]
                elif r_i.origin.getID() == r_j.origin.getID() and r_i.dest.getID() == r_j.dest.getID():
                    lambda_ij_mat.loc[i, j] = -r_i.choice_probability * r_j.choice_probability * theta.loc[
                        r_i.origin.getID(), r_i.dest.getID()]
                else:
                    pass
    return lambda_ij_mat


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


def get_pr(I, R, theta):
    edges_list = set(map(lambda i: i.split('_')[0], I))
    Pr_i = np.zeros((len(edges_list),), dtype=float)
    for idx, i in enumerate(edges_list):
        if not R[i]:
            continue
        pr_i = []
        for r_i in R[i]:
            """theta_ri = theta.loc[r_i.origin.getID(), r_i.dest.getID()]
            if theta_ri == 0:
                pr_i.append(0)
            else:
                pr_i.append(r_i.choice_probability / (r_i.choice_probability * theta_ri))"""
            theta_ri = theta.loc[r_i.origin.getID(), r_i.dest.getID()]
            if theta_ri == 0:
                pr_i.append(0)
            else:
                pr_i.append(1 / theta_ri)

        Pr_i[idx] = np.nanmean(pr_i)
    # Pr_i[i] is the average probability of choosing one of the route passing through link i
    return Pr_i


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

    # Get the traffic counts from virtual sensors and calculate covariance/correlation
    data_hour = det_output_to_df(det_out_xml)
    cov_by_hour = data_hour.cov(numeric_only=True)
    corr_by_hour = data_hour.corr(numeric_only=True).fillna(.0)

    # Where condition is False, keep the original value! So, we're looking at nan
    mask = corr_by_hour.mask((corr_by_hour.abs() > .7) & (corr_by_hour.abs() < 1))

    # element NaN in mask are those for which the condition is true
    correlated_pairs = set()
    for sensor_id in mask.columns:
        # get the correlated sensors
        nan_elem = mask[sensor_id].loc[mask[sensor_id].isna()]
        if nan_elem.empty:
            continue
        corr_sensors = list(zip([sensor_id for _ in range(len(nan_elem.index.tolist()))], nan_elem.index.tolist()))
        [correlated_pairs.add(p) for p in corr_sensors]

    # set of edges ID having sensors (i consider a small subset)
    I = list(sumolib.xml.parse_fast(sensors_csv, 'inductionLoop', ['id']))
    I = list(map(lambda sensor: sensor.id, I))
    w = 0.01

    # filter the traffic counts and OD pairs to the selected links in I
    edges_list = set(map(lambda i: i.split('_')[0], I))
    data_hour = data_hour.filter(edges_list)
    hourly_cov = cov_by_hour  # [ts[0]]
    hourly_cov = hourly_cov.filter(edges_list).T.filter(edges_list)

    create_od_pairs(I, allowed_pairs, initial_solution_path=start_decision_vector_path, taz_path=out_taz_path)

    # Initial traffic demand
    SUMOUtils.create_trips(net_path,
                           taz_path=out_taz_path,
                           odpairs_path=start_decision_vector_path,
                           trips_path=out_trips_path,
                           out_routes_fname=out_routes_path,
                           vtype_add_file_path=vtype_add_file)

    od_pairs_routes_dict, od_counts = map_od_routes(sumolib.net.readNet(net_path),
                                                    start_decision_vector_path,
                                                    out_routes_path)

    for od_pair in tqdm.tqdm(od_pairs_routes_dict):
        od_pairs_routes_dict[od_pair] = calculate_route_choice_probability(od_pairs_routes_dict[od_pair])
    od_counts = {k: v for k, v in od_counts.items() if k in allowed_pairs}

    # from od counts in dict format: {(O1,D1)->[n1]; (O2,D2)->[n2]...} to matrix
    theta = get_dense_od_demand_matrix(I, od_counts)  # type: pd.DataFrame

    # unroll the list of routes
    all_routes = list(itertools.chain(*od_pairs_routes_dict.values()))
    # Calculate R, a dict where KEY is an edge ID (in I), VALUE is a list of routes containing i (for i in I)
    print('calculating initial R')
    R = get_routes_by_edge(I, all_routes)
    print('calculating initial Pr')
    Pr_i = get_pr(I, R, theta)
    lambda_ij = get_lambda_ij(theta, I, R)

    hourly_counts_avg = data_hour.mean(axis=0)
    # first summation
    s1 = ((hourly_counts_avg - (Pr_i * theta).sum(axis=0)) ** 2).sum()
    # second summation
    s2 = ((cov_by_hour - lambda_ij) ** 2).sum().sum()
    objf = s1 + w * s2


    def objfun(y0):
        global iic  # just a counter
        iic += 1

        # map the decision vector y0 to OD pairs
        _od_counts = dict(zip(list(od_counts.keys()), y0))
        theta = get_dense_od_demand_matrix(I, _od_counts)
        Pr_i = get_pr(I, R, theta)
        lambda_ij = get_lambda_ij(theta, I, R)
        s1 = ((hourly_counts_avg - (Pr_i * theta).sum(axis=0)) ** 2).sum()
        s2 = ((cov_by_hour - lambda_ij) ** 2).sum().sum()
        return s1 + w * s2


    bounds = ([0 for _ in list(od_counts.keys())],
              [200 for _ in list(od_counts.keys())])
    # bounds = [(0, 200) for _ in list(od_counts.keys())]

    res = least_squares(objfun, x0=list(od_counts.values()), bounds=bounds, verbose=2, max_nfev=300)
    # from scipy.optimize import minimize, rosen, rosen_der
    # res = minimize(objfun, x0=list(od_counts.values()), bounds=bounds, options={'disp': True,'maxiter':100})

    print('output decision vector: {}'.format(res.x))
    od_counts = dict(zip(list(od_counts.keys()), res.x))

    with open(output_decision_vector_path, 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "data", "datamode_file.xsd")  # noqa
        outf.write(4 * ' ' + '<interval id="%s" begin="%s" end="%s">\n' % (0, time_period_start, time_period_end))
        for od, value in od_counts.items():
            outf.write(8 * ' ' + '<edgeRelation from="%s" to="%s" count="%s"/>\n' % (od[0], od[1], int(value)))
        outf.write(4 * ' ' + '</interval>\n')
        outf.write('</data>\n')

    with open('res.pkl', 'wb') as f:
        pickle.dump(res, f)
