#!/usr/bin/env python3
"""Inspect STEP assembly names and placements without discarding surfaces."""
import argparse
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else '(unnamed)'


def load(path):
    doc = TDocStd_Document(TCollection_ExtendedString('cad'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(str(path))
    if not reader.Transfer(doc):
        raise ValueError('Assembly transfer failed')
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def walk(tool, label, depth=0):
    referred = TDF_Label()
    target = label
    if tool.IsReference_s(label):
        tool.GetReferredShape_s(label, referred)
        target = referred
    location = tool.GetLocation_s(label).Transformation()
    matrix = [[round(location.Value(i, j), 6) for j in range(1, 5)] for i in range(1, 4)]
    print('  '*depth, name(target), 'placement', matrix, flush=True)
    children = TDF_LabelSequence()
    if tool.GetComponents_s(target, children):
        for i in range(1, children.Length()+1):
            walk(tool, children.Value(i), depth+1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    args = parser.parse_args()
    doc, tool = load(args.source)
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length()+1):
        walk(tool, roots.Value(i))
