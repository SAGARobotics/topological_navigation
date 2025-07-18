#!/usr/bin/env python
# -*- coding: utf-8 -*-

import yaml
import argparse

def clean_edges(file_path, output_path):
    # Load the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    # Iterate through all nodes
    for node in data.get('nodes', []):
        node_name = node.get('node', {}).get('name')
        if not node_name:
            continue

        # Filter edges where the 'node' field doesn't match the parent node's name
        edges = node.get('node', {}).get('edges', [])
        filtered_edges = [edge for edge in edges if edge.get('node') != node_name]
        node['node']['edges'] = filtered_edges

    # Save the cleaned YAML back to a file
    with open(output_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert treat_ex.yaml to new treatment format.')
    parser.add_argument('--input', type=str, default='tmap.tmap2',
                        help='Input YAML file (default: tmap.tmap2)')
    parser.add_argument('--output', type=str, default='tmap.tmap2',
                        help='Output YAML file (default: tmap.tmap2)')
    args = parser.parse_args()

    clean_edges(args.input, args.output)

