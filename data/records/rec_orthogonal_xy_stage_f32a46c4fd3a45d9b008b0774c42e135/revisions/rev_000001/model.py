from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.190
BASE_WID = 0.126
BASE_THK = 0.014
BASE_POCKET_LEN = 0.152
BASE_POCKET_WID = 0.048
BASE_POCKET_DEPTH = 0.004

X_RAIL_LEN = 0.154
X_RAIL_W = 0.016
X_RAIL_H = 0.008
X_RAIL_Y = 0.031
X_RAIL_BEVEL = 0.003

X_TRAVEL = 0.028
X_STOP_X = 0.087
X_STOP_Y = -0.057
X_STOP_LEN = 0.008
X_STOP_WID = 0.012
X_STOP_H = 0.012

X_SLIDE_LEN = 0.130
X_SLIDE_WID = 0.100
X_BODY_THK = 0.010
X_UNDERSIDE_RELIEF_LEN = 0.098
X_UNDERSIDE_RELIEF_WID = 0.028
X_UNDERSIDE_RELIEF_DEPTH = 0.006
X_SIDEKEEP_LEN = 0.120
X_SIDEKEEP_WID = 0.008
X_SIDEKEEP_Y = 0.045
X_SIDEKEEP_BOTTOM = -0.006
X_SIDEKEEP_H = 0.010
X_TOP_PAD_LEN = 0.090
X_TOP_PAD_WID = 0.060
X_TOP_PAD_THK = 0.003
X_GIB_LEN = 0.108
X_GIB_WID = 0.008
X_GIB_THK = 0.002
X_GIB_Y = 0.039
X_PAWL_X = 0.049
X_PAWL_Y = -0.051
X_PAWL_BOTTOM = -0.003
X_PAWL_H = 0.010

Y_RAIL_LEN = 0.090
Y_RAIL_W = 0.010
Y_RAIL_H = 0.006
Y_RAIL_X = 0.018
Y_RAIL_BEVEL = 0.002

Y_TRAVEL = 0.017
Y_STOP_X = 0.031
Y_STOP_Y = 0.059
Y_STOP_WID = 0.010
Y_STOP_LEN = 0.008
Y_STOP_BOTTOM = X_BODY_THK + X_TOP_PAD_THK
Y_STOP_H = 0.006
Y_STOP_ARM_LEN = 0.018

Y_SADDLE_WID = 0.072
Y_SADDLE_LEN = 0.106
Y_BODY_THK = 0.010
Y_UNDERSIDE_RELIEF_WID = 0.018
Y_UNDERSIDE_RELIEF_LEN = 0.080
Y_UNDERSIDE_RELIEF_DEPTH = 0.005
Y_SIDEKEEP_WID = 0.006
Y_SIDEKEEP_X = 0.031
Y_SIDEKEEP_BOTTOM = -0.005
Y_SIDEKEEP_H = 0.009
Y_TOP_PAD_WID = 0.056
Y_TOP_PAD_LEN = 0.086
Y_TOP_PAD_THK = 0.003
Y_CLAMP_WID = 0.048
Y_CLAMP_LEN = 0.006
Y_CLAMP_THK = 0.002
Y_CLAMP_Y = 0.040
Y_PAWL_Y = 0.032
Y_PAWL_X = 0.031
Y_PAWL_BOTTOM = -0.004
Y_PAWL_H = 0.008

TOP_PLATE_WID = 0.094
TOP_PLATE_LEN = 0.066
TOP_PLATE_THK = 0.006
TOP_PLATE_WINDOW_W = 0.038
TOP_PLATE_WINDOW_L = 0.024
TOP_PLATE_HOLE_D = 0.0055

BASE_TO_X_Z = BASE_THK + X_RAIL_H
X_TO_Y_Z = X_BODY_THK + X_TOP_PAD_THK + Y_RAIL_H
Y_TO_PLATE_Z = Y_BODY_THK + Y_TOP_PAD_THK


def _union_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _compound_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    return cq.Workplane(obj=cq.Compound.makeCompound([shape.val() for shape in shapes]))


def _cap_screw_heads(
    points: tuple[tuple[float, float], ...],
    *,
    z0: float,
    radius: float = 0.0027,
    height: float = 0.0022,
    socket_radius: float = 0.0012,
    socket_depth: float = 0.0008,
) -> cq.Workplane:
    heads: list[cq.Workplane] = []
    for x_pos, y_pos in points:
        head = cq.Workplane("XY").circle(radius).extrude(height)
        head = (
            head.faces(">Z")
            .workplane(centerOption="CenterOfMass")
            .circle(socket_radius)
            .cutBlind(-socket_depth)
        )
        heads.append(head.translate((x_pos, y_pos, z0)))
    return _compound_shapes(*heads)


def _x_way_rail(length: float, width: float, height: float, bevel: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .moveTo(-width / 2.0, 0.0)
        .lineTo(-(width / 2.0) + bevel, height)
        .lineTo((width / 2.0) - bevel, height)
        .lineTo(width / 2.0, 0.0)
        .close()
        .extrude(length, both=True)
    )


def _y_way_rail(length: float, width: float, height: float, bevel: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(-width / 2.0, 0.0)
        .lineTo(-(width / 2.0) + bevel, height)
        .lineTo((width / 2.0) - bevel, height)
        .lineTo(width / 2.0, 0.0)
        .close()
        .extrude(length, both=True)
    )


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, material: str, name: str) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=name)


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder_visual(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _base_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_LEN, BASE_WID, BASE_THK, centered=(True, True, False))
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_POCKET_LEN, BASE_POCKET_WID)
        .cutBlind(-BASE_POCKET_DEPTH)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.068, -0.044),
                (-0.068, 0.044),
                (0.068, -0.044),
                (0.068, 0.044),
            ]
        )
        .cboreHole(0.005, 0.008, 0.003)
    )
    return body.edges("|Z").chamfer(0.0012)


def _base_x_rails_shape() -> cq.Workplane:
    rails = []
    for y_pos in (-X_RAIL_Y, X_RAIL_Y):
        rail = _x_way_rail(X_RAIL_LEN, X_RAIL_W, X_RAIL_H, X_RAIL_BEVEL)
        rails.append(rail.translate((0.0, y_pos, BASE_THK)))
    return _compound_shapes(*rails)


def _base_stop_shape(sign: float) -> cq.Workplane:
    stop = cq.Workplane("XY").box(X_STOP_LEN, X_STOP_WID, X_STOP_H, centered=(True, True, False))
    stop = stop.faces(">Z").edges().chamfer(0.0008)
    return stop.translate((sign * X_STOP_X, X_STOP_Y, BASE_THK))


def _x_slide_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(X_SLIDE_LEN, X_SLIDE_WID, X_BODY_THK, centered=(True, True, False))
    body = body.cut(
        cq.Workplane("XY")
        .box(
            X_UNDERSIDE_RELIEF_LEN,
            X_UNDERSIDE_RELIEF_WID,
            X_UNDERSIDE_RELIEF_DEPTH,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.0))
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.030), (0.0, 0.030)])
        .rect(X_SLIDE_LEN - 0.026, 0.012)
        .cutBlind(-0.002)
    )
    pad = cq.Workplane("XY").box(X_TOP_PAD_LEN, X_TOP_PAD_WID, X_TOP_PAD_THK, centered=(True, True, False))
    pad = pad.translate((0.0, 0.0, X_BODY_THK - 0.0002))

    keepers = []
    for y_pos in (-X_SIDEKEEP_Y, X_SIDEKEEP_Y):
        keeper = cq.Workplane("XY").box(
            X_SIDEKEEP_LEN,
            X_SIDEKEEP_WID,
            X_SIDEKEEP_H,
            centered=(True, True, False),
        )
        keepers.append(keeper.translate((0.0, y_pos, X_SIDEKEEP_BOTTOM)))

    body = _union_shapes(body, pad, *keepers)
    return body.edges("|Z").chamfer(0.0010)


def _x_gib_strips_shape() -> cq.Workplane:
    strips = []
    screw_points: list[tuple[float, float]] = []
    for y_pos in (-X_GIB_Y, X_GIB_Y):
        strip = cq.Workplane("XY").box(X_GIB_LEN, X_GIB_WID, X_GIB_THK, centered=(True, True, False))
        strips.append(strip.translate((0.0, y_pos, X_BODY_THK)))
        screw_points.extend([(-0.034, y_pos), (0.0, y_pos), (0.034, y_pos)])
    screws = _cap_screw_heads(tuple(screw_points), z0=X_BODY_THK + X_GIB_THK)
    return _compound_shapes(*strips, screws)


def _x_slide_y_rails_shape() -> cq.Workplane:
    rails = []
    for x_pos in (-Y_RAIL_X, Y_RAIL_X):
        rail = _y_way_rail(Y_RAIL_LEN, Y_RAIL_W, Y_RAIL_H, Y_RAIL_BEVEL)
        rails.append(rail.translate((x_pos, 0.0, X_BODY_THK + X_TOP_PAD_THK)))
    return _compound_shapes(*rails)


def _x_pawl_shape(sign: float) -> cq.Workplane:
    pawl = cq.Workplane("XY").box(X_STOP_LEN, X_STOP_WID, X_PAWL_H, centered=(True, True, False))
    pawl = pawl.faces(">Z").edges().chamfer(0.0008)
    return pawl.translate((sign * X_PAWL_X, X_STOP_Y, X_PAWL_BOTTOM))


def _x_slide_y_stop_shape(sign: float) -> cq.Workplane:
    stop = cq.Workplane("XY").box(Y_STOP_WID, Y_STOP_LEN, Y_STOP_H, centered=(True, True, False))
    stop = stop.faces(">Z").edges().chamfer(0.0007)
    return stop.translate((Y_STOP_X, sign * Y_STOP_Y, Y_STOP_BOTTOM))


def _y_saddle_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(Y_SADDLE_WID, Y_SADDLE_LEN, Y_BODY_THK, centered=(True, True, False))
    body = body.cut(
        cq.Workplane("XY")
        .box(
            Y_UNDERSIDE_RELIEF_WID,
            Y_UNDERSIDE_RELIEF_LEN,
            Y_UNDERSIDE_RELIEF_DEPTH,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.0))
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.033), (0.0, 0.033)])
        .rect(0.040, 0.012)
        .cutBlind(-0.002)
    )
    pad = cq.Workplane("XY").box(Y_TOP_PAD_WID, Y_TOP_PAD_LEN, Y_TOP_PAD_THK, centered=(True, True, False))
    pad = pad.translate((0.0, 0.0, Y_BODY_THK - 0.0002))

    keepers = []
    for x_pos in (-Y_SIDEKEEP_X, Y_SIDEKEEP_X):
        keeper = cq.Workplane("XY").box(
            Y_SIDEKEEP_WID,
            Y_SADDLE_LEN - 0.012,
            Y_SIDEKEEP_H,
            centered=(True, True, False),
        )
        keepers.append(keeper.translate((x_pos, 0.0, Y_SIDEKEEP_BOTTOM)))

    body = _union_shapes(body, pad, *keepers)
    return body.edges("|Z").chamfer(0.0009)


def _y_clamp_strips_shape() -> cq.Workplane:
    clamps = []
    screw_points: list[tuple[float, float]] = []
    for y_pos in (-Y_CLAMP_Y, Y_CLAMP_Y):
        clamp = cq.Workplane("XY").box(Y_CLAMP_WID, Y_CLAMP_LEN, Y_CLAMP_THK, centered=(True, True, False))
        clamps.append(clamp.translate((0.0, y_pos, Y_TO_PLATE_Z)))
        screw_points.extend([(-0.014, y_pos), (0.014, y_pos)])
    screws = _cap_screw_heads(tuple(screw_points), z0=Y_TO_PLATE_Z + Y_CLAMP_THK, radius=0.0024, height=0.0020)
    return _compound_shapes(*clamps, screws)


def _y_pawl_shape(sign: float) -> cq.Workplane:
    pawl = cq.Workplane("XY").box(Y_STOP_WID, Y_STOP_LEN, Y_PAWL_H, centered=(True, True, False))
    pawl = pawl.faces(">Z").edges().chamfer(0.0007)
    return pawl.translate((Y_PAWL_X, sign * Y_PAWL_Y, Y_PAWL_BOTTOM))


def _top_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TOP_PLATE_WID, TOP_PLATE_LEN, TOP_PLATE_THK, centered=(True, True, False))
    plate = plate.edges("|Z").chamfer(0.0008)
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(TOP_PLATE_WINDOW_W, TOP_PLATE_WINDOW_L)
        .cutThruAll()
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.028, -0.018),
                (-0.028, 0.018),
                (0.028, -0.018),
                (0.028, 0.018),
                (-0.040, 0.0),
                (0.040, 0.0),
            ]
        )
        .hole(TOP_PLATE_HOLE_D)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope_translation_table")

    model.material("base_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("machined_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("saddle_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("ground_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("plate_black", rgba=(0.09, 0.10, 0.11, 1.0))

    base = model.part("base")
    _add_box_visual(base, (BASE_LEN, BASE_WID, BASE_THK), (0.0, 0.0, BASE_THK / 2.0), "base_black", "base_body")
    _add_box_visual(
        base,
        (BASE_POCKET_LEN, BASE_POCKET_WID, 0.002),
        (0.0, 0.0, BASE_THK + 0.001),
        "base_black",
        "center_land",
    )
    _add_box_visual(
        base,
        (X_RAIL_LEN, X_RAIL_W, X_RAIL_H),
        (0.0, -X_RAIL_Y, BASE_THK + X_RAIL_H / 2.0),
        "ground_steel",
        "x_rail_neg",
    )
    _add_box_visual(
        base,
        (X_RAIL_LEN, X_RAIL_W, X_RAIL_H),
        (0.0, X_RAIL_Y, BASE_THK + X_RAIL_H / 2.0),
        "ground_steel",
        "x_rail_pos",
    )
    _add_box_visual(
        base,
        (X_STOP_LEN, X_STOP_WID, X_STOP_H),
        (-X_STOP_X, X_STOP_Y, BASE_THK + X_STOP_H / 2.0),
        "ground_steel",
        "x_stop_neg",
    )
    _add_box_visual(
        base,
        (X_STOP_LEN, X_STOP_WID, X_STOP_H),
        (X_STOP_X, X_STOP_Y, BASE_THK + X_STOP_H / 2.0),
        "ground_steel",
        "x_stop_pos",
    )
    for index, (x_pos, y_pos) in enumerate(((-0.068, -0.044), (-0.068, 0.044), (0.068, -0.044), (0.068, 0.044)), start=1):
        _add_cylinder_visual(
            base,
            radius=0.0031,
            length=0.0024,
            xyz=(x_pos, y_pos, BASE_THK + 0.0012),
            material="ground_steel",
            name=f"mount_screw_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WID, BASE_TO_X_Z + 0.004)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, (BASE_TO_X_Z + 0.004) / 2.0)),
    )

    x_slide = model.part("x_slide")
    _add_box_visual(
        x_slide,
        (X_SLIDE_LEN, X_SLIDE_WID, X_BODY_THK),
        (0.0, 0.0, X_BODY_THK / 2.0),
        "machined_aluminum",
        "x_body",
    )
    _add_box_visual(
        x_slide,
        (X_TOP_PAD_LEN, X_TOP_PAD_WID, X_TOP_PAD_THK),
        (0.0, 0.0, X_BODY_THK + X_TOP_PAD_THK / 2.0),
        "machined_aluminum",
        "x_top_pad",
    )
    _add_box_visual(
        x_slide,
        (X_SIDEKEEP_LEN, X_SIDEKEEP_WID, X_SIDEKEEP_H),
        (0.0, -X_SIDEKEEP_Y, X_SIDEKEEP_BOTTOM + X_SIDEKEEP_H / 2.0),
        "machined_aluminum",
        "x_keeper_neg",
    )
    _add_box_visual(
        x_slide,
        (X_SIDEKEEP_LEN, X_SIDEKEEP_WID, X_SIDEKEEP_H),
        (0.0, X_SIDEKEEP_Y, X_SIDEKEEP_BOTTOM + X_SIDEKEEP_H / 2.0),
        "machined_aluminum",
        "x_keeper_pos",
    )
    _add_box_visual(
        x_slide,
        (X_GIB_LEN, X_GIB_WID, X_GIB_THK),
        (0.0, -X_GIB_Y, X_BODY_THK + X_GIB_THK / 2.0),
        "ground_steel",
        "x_gib_neg",
    )
    _add_box_visual(
        x_slide,
        (X_GIB_LEN, X_GIB_WID, X_GIB_THK),
        (0.0, X_GIB_Y, X_BODY_THK + X_GIB_THK / 2.0),
        "ground_steel",
        "x_gib_pos",
    )
    for y_suffix, y_pos in (("neg", -X_GIB_Y), ("pos", X_GIB_Y)):
        for index, x_pos in enumerate((-0.034, 0.0, 0.034), start=1):
            _add_cylinder_visual(
                x_slide,
                radius=0.0027,
                length=0.0022,
                xyz=(x_pos, y_pos, X_BODY_THK + X_GIB_THK + 0.0011),
                material="ground_steel",
                name=f"x_gib_screw_{y_suffix}_{index}",
            )
    _add_box_visual(
        x_slide,
        (Y_RAIL_W, Y_RAIL_LEN, Y_RAIL_H),
        (-Y_RAIL_X, 0.0, X_TO_Y_Z - Y_RAIL_H / 2.0),
        "ground_steel",
        "y_rail_neg",
    )
    _add_box_visual(
        x_slide,
        (Y_RAIL_W, Y_RAIL_LEN, Y_RAIL_H),
        (Y_RAIL_X, 0.0, X_TO_Y_Z - Y_RAIL_H / 2.0),
        "ground_steel",
        "y_rail_pos",
    )
    _add_box_visual(
        x_slide,
        (X_STOP_LEN, X_STOP_WID, X_PAWL_H),
        (-X_PAWL_X, X_PAWL_Y, X_PAWL_BOTTOM + X_PAWL_H / 2.0),
        "ground_steel",
        "x_pawl_neg",
    )
    _add_box_visual(
        x_slide,
        (X_STOP_LEN, X_STOP_WID, X_PAWL_H),
        (X_PAWL_X, X_PAWL_Y, X_PAWL_BOTTOM + X_PAWL_H / 2.0),
        "ground_steel",
        "x_pawl_pos",
    )
    _add_box_visual(
        x_slide,
        (Y_STOP_WID, Y_STOP_ARM_LEN, Y_STOP_H),
        (Y_STOP_X, -0.050, Y_STOP_BOTTOM + Y_STOP_H / 2.0),
        "ground_steel",
        "y_stop_bracket_neg",
    )
    _add_box_visual(
        x_slide,
        (Y_STOP_WID, Y_STOP_ARM_LEN, Y_STOP_H),
        (Y_STOP_X, 0.050, Y_STOP_BOTTOM + Y_STOP_H / 2.0),
        "ground_steel",
        "y_stop_bracket_pos",
    )
    _add_box_visual(
        x_slide,
        (Y_STOP_WID, Y_STOP_LEN, Y_STOP_H),
        (Y_STOP_X, -Y_STOP_Y, Y_STOP_BOTTOM + Y_STOP_H / 2.0),
        "ground_steel",
        "y_stop_neg",
    )
    _add_box_visual(
        x_slide,
        (Y_STOP_WID, Y_STOP_LEN, Y_STOP_H),
        (Y_STOP_X, Y_STOP_Y, Y_STOP_BOTTOM + Y_STOP_H / 2.0),
        "ground_steel",
        "y_stop_pos",
    )
    x_slide.inertial = Inertial.from_geometry(
        Box((X_SLIDE_LEN, X_SLIDE_WID, X_TO_Y_Z - X_SIDEKEEP_BOTTOM)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, (X_TO_Y_Z + X_SIDEKEEP_BOTTOM) / 2.0)),
    )

    y_saddle = model.part("y_saddle")
    _add_box_visual(
        y_saddle,
        (Y_SADDLE_WID, Y_SADDLE_LEN, Y_BODY_THK),
        (0.0, 0.0, Y_BODY_THK / 2.0),
        "saddle_gray",
        "y_body",
    )
    _add_box_visual(
        y_saddle,
        (Y_TOP_PAD_WID, Y_TOP_PAD_LEN, Y_TOP_PAD_THK),
        (0.0, 0.0, Y_BODY_THK + Y_TOP_PAD_THK / 2.0),
        "saddle_gray",
        "y_top_pad",
    )
    _add_box_visual(
        y_saddle,
        (Y_SIDEKEEP_WID, Y_SADDLE_LEN - 0.012, Y_SIDEKEEP_H),
        (-Y_SIDEKEEP_X, 0.0, Y_SIDEKEEP_BOTTOM + Y_SIDEKEEP_H / 2.0),
        "saddle_gray",
        "y_keeper_neg",
    )
    _add_box_visual(
        y_saddle,
        (Y_SIDEKEEP_WID, Y_SADDLE_LEN - 0.012, Y_SIDEKEEP_H),
        (Y_SIDEKEEP_X, 0.0, Y_SIDEKEEP_BOTTOM + Y_SIDEKEEP_H / 2.0),
        "saddle_gray",
        "y_keeper_pos",
    )
    _add_box_visual(
        y_saddle,
        (Y_STOP_WID, Y_STOP_LEN, Y_PAWL_H),
        (Y_PAWL_X, -Y_PAWL_Y, Y_PAWL_BOTTOM + Y_PAWL_H / 2.0),
        "ground_steel",
        "y_pawl_neg",
    )
    _add_box_visual(
        y_saddle,
        (Y_STOP_WID, Y_STOP_LEN, Y_PAWL_H),
        (Y_PAWL_X, Y_PAWL_Y, Y_PAWL_BOTTOM + Y_PAWL_H / 2.0),
        "ground_steel",
        "y_pawl_pos",
    )
    _add_box_visual(
        y_saddle,
        (Y_CLAMP_WID, Y_CLAMP_LEN, Y_CLAMP_THK),
        (0.0, -Y_CLAMP_Y, Y_TO_PLATE_Z + Y_CLAMP_THK / 2.0),
        "ground_steel",
        "y_clamp_neg",
    )
    _add_box_visual(
        y_saddle,
        (Y_CLAMP_WID, Y_CLAMP_LEN, Y_CLAMP_THK),
        (0.0, Y_CLAMP_Y, Y_TO_PLATE_Z + Y_CLAMP_THK / 2.0),
        "ground_steel",
        "y_clamp_pos",
    )
    for y_suffix, y_pos in (("neg", -Y_CLAMP_Y), ("pos", Y_CLAMP_Y)):
        for index, x_pos in enumerate((-0.014, 0.014), start=1):
            _add_cylinder_visual(
                y_saddle,
                radius=0.0024,
                length=0.0020,
                xyz=(x_pos, y_pos, Y_TO_PLATE_Z + Y_CLAMP_THK + 0.0010),
                material="ground_steel",
                name=f"y_clamp_screw_{y_suffix}_{index}",
            )
    y_saddle.inertial = Inertial.from_geometry(
        Box((Y_SADDLE_WID, Y_SADDLE_LEN, Y_TO_PLATE_Z - Y_SIDEKEEP_BOTTOM)),
        mass=0.46,
        origin=Origin(xyz=(0.0, 0.0, (Y_TO_PLATE_Z + Y_SIDEKEEP_BOTTOM) / 2.0)),
    )

    top_plate = model.part("top_plate")
    _add_mesh_visual(top_plate, _top_plate_shape(), "top_plate", "plate_black", "plate")
    top_plate.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_WID, TOP_PLATE_LEN, TOP_PLATE_THK)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_THK / 2.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_X_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=140.0,
            velocity=0.06,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_saddle,
        origin=Origin(xyz=(0.0, 0.0, X_TO_Y_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=90.0,
            velocity=0.05,
        ),
    )
    model.articulation(
        "y_to_plate",
        ArticulationType.FIXED,
        parent=y_saddle,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, Y_TO_PLATE_Z)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_saddle = object_model.get_part("y_saddle")
    top_plate = object_model.get_part("top_plate")
    x_axis = object_model.get_articulation("base_to_x")
    y_axis = object_model.get_articulation("x_to_y")
    top_mount = object_model.get_articulation("y_to_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "x_axis_is_prismatic_x",
        x_axis.articulation_type == ArticulationType.PRISMATIC and tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        f"expected X slide prismatic axis along +X, got type={x_axis.articulation_type} axis={x_axis.axis}",
    )
    ctx.check(
        "y_axis_is_prismatic_y",
        y_axis.articulation_type == ArticulationType.PRISMATIC and tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        f"expected Y saddle prismatic axis along +Y, got type={y_axis.articulation_type} axis={y_axis.axis}",
    )
    ctx.check(
        "plate_mount_is_fixed",
        top_mount.articulation_type == ArticulationType.FIXED,
        f"expected fixed plate mount, got {top_mount.articulation_type}",
    )

    ctx.expect_contact(base, x_slide, contact_tol=0.0008, name="base_supports_x_slide")
    ctx.expect_overlap(base, x_slide, axes="xy", min_overlap=0.080, name="x_slide_overlaps_base_footprint")
    ctx.expect_contact(x_slide, y_saddle, contact_tol=0.0008, name="x_slide_supports_y_saddle")
    ctx.expect_overlap(x_slide, y_saddle, axes="xy", min_overlap=0.050, name="y_saddle_overlaps_x_slide_footprint")
    ctx.expect_contact(y_saddle, top_plate, contact_tol=0.0008, name="top_plate_mounts_to_saddle")
    ctx.expect_overlap(y_saddle, top_plate, axes="xy", min_overlap=0.050, name="top_plate_overlaps_saddle_pad")

    with ctx.pose({x_axis: X_TRAVEL}):
        ctx.expect_gap(
            base,
            x_slide,
            axis="x",
            positive_elem="x_stop_pos",
            negative_elem="x_pawl_pos",
            min_gap=0.0015,
            max_gap=0.0035,
            name="x_positive_stop_clearance",
        )
    with ctx.pose({x_axis: -X_TRAVEL}):
        ctx.expect_gap(
            x_slide,
            base,
            axis="x",
            positive_elem="x_pawl_neg",
            negative_elem="x_stop_neg",
            min_gap=0.0015,
            max_gap=0.0035,
            name="x_negative_stop_clearance",
        )
    with ctx.pose({y_axis: Y_TRAVEL}):
        ctx.expect_gap(
            x_slide,
            y_saddle,
            axis="y",
            positive_elem="y_stop_pos",
            negative_elem="y_pawl_pos",
            min_gap=0.0015,
            max_gap=0.0035,
            name="y_positive_stop_clearance",
        )
    with ctx.pose({y_axis: -Y_TRAVEL}):
        ctx.expect_gap(
            y_saddle,
            x_slide,
            axis="y",
            positive_elem="y_pawl_neg",
            negative_elem="y_stop_neg",
            min_gap=0.0015,
            max_gap=0.0035,
            name="y_negative_stop_clearance",
        )

    with ctx.pose({x_axis: X_TRAVEL, y_axis: Y_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_positive_limits")
    with ctx.pose({x_axis: -X_TRAVEL, y_axis: -Y_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_negative_limits")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
