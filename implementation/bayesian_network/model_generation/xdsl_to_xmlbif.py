#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
#!/usr/bin/env python3.8

from xml.etree import ElementTree
from xml.dom import minidom
from typing import List
from dataclasses import dataclass

import argparse
import os


PATH = os.path.dirname(os.path.abspath(__file__))
"""str : Absolute directory path to this file's directory."""


@dataclass
class XDSLNode:
    """A Bayesian network node data class.

    Attributes
    ----------
    node_id : str
        The name/identifier of this specific node.
    states : List[str]
        The names/identifiers of the node's states.
    parents : List[str]
        The names/identifiers of the node's direct parents.
    unwrapped_cpt : List[float]
        The node's conditional probability table unwrapped in a list. The used order is identical to the order in the
        .xmlbif and .xdsl files.
    """

    node_id: str
    states: List[str]
    parents: List[str]
    unwrapped_cpt: List[float]


def main() -> None:
    """Parses the given Bayesian network .xdsl file and creates corresponding .xmlbif file (stored in this file's
    directory). The .xdsl file must be located in this file's directory.

    Note
    ----
    Example start command for the Bayesian network with the name TestSituationClassOne:
    python xdsl_to_xmlbif.py -fn TestSituationClassOne
    Type `python xdsl_to_xmlbif.py --help` for more details on the console arguments.
    """
    parser = set_up_argparser()
    file_name = get_file_name(parser)
    nodes = parse_xdsl_file(file_name)
    xmlbif_content_root = create_xmlbif_content(nodes)
    write_xmlbif_file(xmlbif_content_root, file_name)


def set_up_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BayesFusion .xdsl file format to .xmlbif file format transformation.")
    parser.add_argument("-fn", "--fileName", help="Name of the .xdsl file", action="store", dest="file_name",
                        required=True, type=str)

    return parser


def get_file_name(parser: argparse.ArgumentParser) -> str:
    file_name = parser.parse_args().file_name

    if len(file_name) > 5 and file_name[-5:] == ".xdsl":
        file_name = file_name[:-5]

    return file_name


def parse_xdsl_file(file_name: str) -> List[XDSLNode]:
    tree = ElementTree.parse(f"{PATH}/{file_name}.xdsl")
    root = tree.getroot()  # smile-level
    nodes = [child for child in root if child.tag == "nodes"][0]
    stored_nodes: List[XDSLNode] = []

    for node in nodes:
        node_id = node.attrib["id"]
        state_ids = []
        parent_ids = []
        unwrapped_cpt = []

        for child in node:
            if child.tag == "state":
                state_ids.append(child.attrib["id"])
            elif child.tag == "parents":
                if child.text:
                    parent_ids = [id for id in child.text.split(" ")]
            elif child.tag == "probabilities":
                if child.text:
                    unwrapped_cpt = [float(value) for value in child.text.split(" ")]

        xdsl_node = XDSLNode(node_id, state_ids, parent_ids, unwrapped_cpt)
        stored_nodes.append(xdsl_node)

    return stored_nodes


def create_xmlbif_content(nodes: List[XDSLNode]) -> ElementTree.Element:
    """Creates the corresponding xml tree for the xmlbif file based on the given XDSL nodes.

    Parameters
    ----------
    nodes : List[XDSLNode]
        List comprising all nodes of the input Bayesian network.

    Returns
    -------
    ElementTree.Element
        Root element of the xml tree for the xmlbif file.
    """
    xml_content_root = ElementTree.Element("BIF", attrib={"VERSION": "0.3"})
    xml_content_network = ElementTree.SubElement(xml_content_root, "NETWORK")
    name_element = ElementTree.SubElement(xml_content_network, "NAME")
    name_element.text = "UNTITLED"

    for node in nodes:
        node_root = ElementTree.SubElement(xml_content_network, "VARIABLE", attrib={"TYPE": "nature"})
        name = ElementTree.SubElement(node_root, "NAME")
        name.text = node.node_id

        for state in node.states:
            outcome = ElementTree.SubElement(node_root, "OUTCOME")
            outcome.text = str(state)

        ElementTree.SubElement(node_root, "PROPERTY")

    for node in nodes:
        definition_root = ElementTree.SubElement(xml_content_network, "DEFINITION")
        for_node = ElementTree.SubElement(definition_root, "FOR")
        for_node.text = node.node_id

        for parent in node.parents:
            given_node = ElementTree.SubElement(definition_root, "GIVEN")
            given_node.text = str(parent)

        table = ElementTree.SubElement(definition_root, "TABLE")
        table.text = " ".join([str(value) for value in node.unwrapped_cpt]) + " "

    return xml_content_root


def write_xmlbif_file(xmlbif_content_root: ElementTree.Element, file_name: str) -> None:
    xmlbif_as_string = ElementTree.tostring(xmlbif_content_root, encoding="UTF-8", xml_declaration=True)
    xmlbif_as_string = xmlbif_as_string.decode("utf-8")

    # required to adjust empty elements to align with the convention in
    # the xmlbif files in pgmpy
    xmlbif_as_string = xmlbif_as_string.replace(" />", "/>")

    # etree is only able to write the xml content in a single line
    # xml.dom required to prettify the string
    reparsed_xmlbif = minidom.parseString(xmlbif_as_string)
    xmlbif_as_prettyfied_string = reparsed_xmlbif.toprettyxml(indent="  ", encoding="UTF-8").decode("utf-8")

    with open(f"{PATH}/{file_name}.xmlbif", "w") as file:
        file.write(xmlbif_as_prettyfied_string)


if __name__ == "__main__":
    main()
