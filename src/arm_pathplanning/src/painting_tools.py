import dxfgrabber


class PaintingAction:
    class Color:
        def __init__(self, color):
            self.color = color

        def __repr__(self):
            return "Color({})".format(self.color)

    class Stroke:
        def __init__(self, stroke):
            self.stroke = stroke

        def __repr__(self):
            return "Stroke({})".format(str(self.stroke))


class InvalidPaintingFileError(Exception):
    pass


def read_dxf(filename):
    dxf = dxfgrabber.readfile(filename)
    polylines = [poly for poly in dxf.entities if type(poly) is dxfgrabber.dxfentities.LWPolyline]
    paths = []
    for line in polylines:
        path = []
        for pt in line:
            path += [(pt[0]/1000.0, pt[1]/1000.0)]
        paths += [path]
    return paths


def read_painting(filename):
    dxf = dxfgrabber.readfile(filename)
    polylines = [poly for poly in dxf.entities if type(poly) is dxfgrabber.dxfentities.LWPolyline]

    painting_info = {}
    
    for line in polylines:
        sid_marker, stroke_id, cid_marker, color_id = line.layer.split("_")
        if sid_marker != "STROKEID" or cid_marker != "COLORID":
            print("found polyline on invalid layer")
            raise InvalidPaintingFileError()
        try:
            stroke_id = int(stroke_id)
            color_id = int(color_id)
        except ValueError:
            print("polyline layer '{}' stroke or color ID invalid!".format(line.layer))
            raise InvalidPaintingFileError()

        if stroke_id in painting_info:
            if painting_info[stroke_id]['color'] != color_id:
                print("found multiple layers with same stroke id but different color id!")
                raise InvalidPaintingError()
            painting_info[stroke_id]['lines'].append(line)
        else:
            painting_info[stroke_id] = {'lines': [line],
                                        'color': color_id}

    return [painting_info[i] for i in sorted(list(painting_info))]


def to_action_sequence(painting_list):
    actions = []
    for i, item in enumerate(painting_list):
        color = item['color']
        lines = item['lines']
        if i == 0 and color == 0:
            print("first stroke should specify color")
            raise InvalidPaintingFileError()

        if color != 0:
            actions.append(PaintingAction.Color(color))

        paths = []
        for line in lines:
            path = []
            for pt in line:
                path.append((pt[0]/1000.0, pt[1]/1000.0))
            paths.append(path)
        if not isinstance(actions[-1], PaintingAction.Stroke):
            # After a color change, make a new stroke action
            actions.append(PaintingAction.Stroke(paths))
        else:
            # If color was not changed, just append to existing
            # stroke action
            actions[-1].stroke.extend(paths)
    return actions
