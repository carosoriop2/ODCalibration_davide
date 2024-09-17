from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import sumolib as sumolib
from dataclass_wizard import JSONWizard

sys.setrecursionlimit(100000)

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools', 'route'))
    import route2OD
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

iic = 0


@dataclass
class Route(JSONWizard):
    id: int
    edges: list[sumolib.net.edge]
    origin: sumolib.net.edge
    dest: sumolib.net.edge
    choice_probability: float
    __edges_id: list[str] = field(init=False)

    def __post_init__(self):
        self.__edges_id = list(map(lambda e: e.getID(), self.edges))

    def is_same(self, l: Route) -> bool:
        if len(set(self.__edges_id).difference(l.__edges_id)) == 0:
            return True
        return False

    def has_edge(self, e_id: str) -> bool:
        return e_id in self.__edges_id

    @classmethod
    def with_probability(cls, r: Route, p: float) -> Route:
        return Route(r.id, r.edges, r.origin, r.dest, p)


def create_od_pairs(correlated_pairs, out_fname="init_odpairs.xml", time_period_start=61200, time_period_end=64800):
    # 17h to 18h by default
    with open(out_fname, 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "data", "datamode_file.xsd")  # noqa
        outf.write(4 * ' ' + '<interval id="%s" begin="%s" end="%s">\n' % (0, time_period_start, time_period_end))
        for od_pair in correlated_pairs:
            outf.write(
                8 * ' ' + '<tazRelation from="taz_%s" to="taz_%s" count="%s"/>\n' % (od_pair[0], od_pair[1], str(1)))
        outf.write(4 * ' ' + '</interval>\n')
        outf.write('</data>\n')


def create_trips(net_path, taz_path="taz.xml", odpairs_path="init_odpairs.xml", trips_path="trips.xml",
                 out_routes_fname="routes.xml", vtype_add_file_path="vtype.add.xml"):
    # OD -> Trips
    od2trips_cmd = (f"od2trips -n {taz_path} "
                    f" -z {odpairs_path} "
                    f" -o {trips_path}"
                    f" --vtype passenger")
    print(od2trips_cmd)
    os.system(od2trips_cmd)
    # Trips -> Routes
    duarouter_cmd = (f"duarouter -n {net_path}"
                     f" -a {vtype_add_file_path}"
                     f" -r {trips_path}"
                     f" -o {out_routes_fname}"
                     f" --ignore-errors")
    print(duarouter_cmd)
    os.system(duarouter_cmd)


def create_taz(od_routes: set[(str, str)], od_pairs_lane=False):
    taz_id = 0
    with open("taz.xml", 'w') as outf:
        sumolib.writeXMLHeader(outf, "$Id$", "additional")  # noqa
        written_pairs = set()

        for od in od_routes:
            o = od[0] if od_pairs_lane else od[0].split('_')[0]
            d = od[1] if od_pairs_lane else od[1].split('_')[0]
            if o in written_pairs:
                continue
            written_pairs.add(o)
            """outf.write("    <taz id=\"taz_{}\">\n".format(od[0]))
            outf.write("        <tazSource id=\"{}\" weight=\"1.00\"/>\n".format(od[0]))
            outf.write("    </taz>\n")
            taz_id += 1
            outf.write("    <taz id=\"taz_{}\">\n".format(od[1]))
            outf.write("        <tazSink id=\"{}\" weight=\"1.00\"/>\n".format(od[1]))
            outf.write("    </taz>\n")"""
            outf.write("    <taz id=\"taz_{}\" edges=\"{}\"/>\n".format(o, o))
            # outf.write("    <taz id=\"taz_{}\" edges=\"{}\"/>\n".format(od[0], od[0]))
            # <taz id="<TAZ_ID>" edges="<EDGE_ID> <EDGE_ID> ..."/>
            taz_id += 1

        outf.write("</additional>\n")


def routes2OD(route_fname: str, od_pairs_out_fname: str, aggr_interval=3600) -> None:
    """
    Calculate OD pairs from a set of routes (in SUMO format)
    :param route_fname:
    :param od_pairs_out_fname:
    :param aggr_interval:
    :return:
    """
    opts = [
        "-r", route_fname,
        "-o", od_pairs_out_fname,
        "-i", str(aggr_interval),
        "--edge-relations"
    ]
    route2OD.main(route2OD.get_options(opts))
