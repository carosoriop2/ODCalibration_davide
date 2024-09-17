from pathlib import Path
import os
import pandas as pd
import xmltodict


def detectors_out_to_df(detectors_out_xml):
    with open(detectors_out_xml) as fd:
        doc = xmltodict.parse(fd.read())
        data = doc['detector']['interval']
        detectors_data_df = pd.DataFrame(data).fillna(0)
    return detectors_data_df

def detectors_out_to_table(sim_data_df: pd.DataFrame, field_name: str) -> pd.DataFrame:
    """
    PLEASE NOTE: the lane ID and the sensors ID must match in order to avoid errors

    :param sim_data_df: the DATAFRAME obtained from the virtual detectors output.
    :param field_name: the name of the parameters to be used for the output data table
    :return:
    """
    list_osm_edges = sim_data_df['@id'].unique()
    detectors_name = set(list_osm_edges)
    data_dict = {}
    for osm_id in detectors_name:
        data = sim_data_df.loc[sim_data_df['@id'] == osm_id][field_name]
        data_dict[osm_id] = data.to_numpy()
    data_df_formatted = pd.DataFrame.from_dict(data_dict)

    data_df_formatted['ts_sumo'] = sim_data_df['@begin'].unique()
    data_df_formatted = data_df_formatted.set_index('ts_sumo')
    return data_df_formatted


if __name__ == '__main__':
    fname = "detector.out.xml"
    p = Path(fname)
    df = detectors_out_to_df(fname)
    detectors_out_to_table(df, "@nVehEntered").to_csv(os.path.join(p.parent, "veh_counts.csv"), sep=';')
    detectors_out_to_table(df, "@speed").to_csv(os.path.join(p.parent, "veh_speed.csv"), sep=';')
