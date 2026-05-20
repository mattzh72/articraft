from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_box(geom: MeshGeometry, center, size) -> None:
    cx, cy, cz = center
    sx, sy, sz = size
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = cz - sz / 2.0, cz + sz / 2.0
    start = len(geom.vertices)
    vertices = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for a, b, c in [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]:
        geom.add_face(start + a, start + b, start + c)


def _add_tapered_plate(
    geom: MeshGeometry,
    *,
    x0: float,
    x1: float,
    y: float,
    thickness: float,
    height0: float,
    height1: float,
) -> None:
    y0, y1 = y - thickness / 2.0, y + thickness / 2.0
    start = len(geom.vertices)
    vertices = [
        (x0, y0, -height0 / 2.0),
        (x1, y0, -height1 / 2.0),
        (x1, y1, -height1 / 2.0),
        (x0, y1, -height0 / 2.0),
        (x0, y0, height0 / 2.0),
        (x1, y0, height1 / 2.0),
        (x1, y1, height1 / 2.0),
        (x0, y1, height0 / 2.0),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for a, b, c in [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]:
        geom.add_face(start + a, start + b, start + c)


def _link_plate_geometry(
    *,
    length: float,
    y_side: float,
    plate_thickness: float,
    start_x: float,
    root_height: float,
    tip_height: float,
    bridge_specs,
    web_specs,
) -> MeshGeometry:
    geom = MeshGeometry()
    _add_tapered_plate(
        geom,
        x0=start_x,
        x1=length,
        y=y_side,
        thickness=plate_thickness,
        height0=root_height,
        height1=tip_height,
    )
    _add_tapered_plate(
        geom,
        x0=start_x,
        x1=length,
        y=-y_side,
        thickness=plate_thickness,
        height0=root_height,
        height1=tip_height,
    )
    for center, size in bridge_specs:
        _add_box(geom, center, size)
    for center, size in web_specs:
        _add_box(geom, center, size)
    return geom


def _y_cylinder_origin(x: float, y: float, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_finger_module")

    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    gunmetal = model.material("gunmetal_plates", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber_rib = model.material("rubber_rib", rgba=(0.05, 0.05, 0.045, 1.0))

    root = model.part("root_knuckle")
    root.visual(
        Box((0.105, 0.135, 0.060)),
        origin=Origin(xyz=(-0.097, 0.0, -0.022)),
        material=dark_anodized,
        name="mounting_block",
    )
    for i, y in enumerate((-0.048, 0.048)):
        root.visual(
            Box((0.080, 0.036, 0.090)),
            origin=Origin(xyz=(-0.023, y, 0.0)),
            material=dark_anodized,
            name=f"root_cheek_{i}",
        )
    root.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=_y_cylinder_origin(0.0, -0.048),
        material=brushed_steel,
        name="root_barrel_0",
    )
    root.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=_y_cylinder_origin(0.0, 0.048),
        material=brushed_steel,
        name="root_barrel_1",
    )
    for i, (x, y) in enumerate(
        [(-0.125, -0.043), (-0.125, 0.043), (-0.070, -0.043), (-0.070, 0.043)]
    ):
        root.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(x, y, 0.011)),
            material=brushed_steel,
            name=f"mount_bolt_{i}",
        )

    middle = model.part("middle_link")
    middle_len = 0.170
    middle.visual(
        mesh_from_geometry(
            _link_plate_geometry(
                length=middle_len,
                y_side=0.055,
                plate_thickness=0.010,
                start_x=0.034,
                root_height=0.070,
                tip_height=0.050,
                bridge_specs=[
                    ((0.065, 0.0, 0.0), (0.018, 0.116, 0.020)),
                    ((0.132, 0.0, 0.0), (0.016, 0.112, 0.018)),
                ],
                web_specs=[
                    ((0.036, 0.0, 0.0), (0.070, 0.052, 0.026)),
                ],
            ),
            "middle_side_plates",
        ),
        material=gunmetal,
        name="side_plate_frame",
    )
    middle.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=_y_cylinder_origin(0.0, 0.0),
        material=brushed_steel,
        name="proximal_barrel",
    )
    middle.visual(
        Cylinder(radius=0.026, length=0.034),
        origin=_y_cylinder_origin(middle_len, -0.049),
        material=brushed_steel,
        name="distal_barrel_0",
    )
    middle.visual(
        Cylinder(radius=0.026, length=0.034),
        origin=_y_cylinder_origin(middle_len, 0.049),
        material=brushed_steel,
        name="distal_barrel_1",
    )

    distal = model.part("distal_link")
    distal_len = 0.105
    distal.visual(
        mesh_from_geometry(
            _link_plate_geometry(
                length=distal_len,
                y_side=0.050,
                plate_thickness=0.009,
                start_x=0.030,
                root_height=0.052,
                tip_height=0.042,
                bridge_specs=[
                    ((0.055, 0.0, 0.0), (0.014, 0.106, 0.017)),
                    ((0.072, 0.0, 0.0), (0.012, 0.104, 0.016)),
                ],
                web_specs=[
                    ((0.030, 0.0, 0.0), (0.060, 0.046, 0.023)),
                ],
            ),
            "distal_side_plates",
        ),
        material=gunmetal,
        name="side_plate_frame",
    )
    distal.visual(
        Cylinder(radius=0.024, length=0.064),
        origin=_y_cylinder_origin(0.0, 0.0),
        material=brushed_steel,
        name="proximal_barrel",
    )
    distal.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=_y_cylinder_origin(distal_len, -0.046),
        material=brushed_steel,
        name="distal_barrel_0",
    )
    distal.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=_y_cylinder_origin(distal_len, 0.046),
        material=brushed_steel,
        name="distal_barrel_1",
    )

    pad = model.part("pad_tip")
    pad_frame = MeshGeometry()
    _add_box(pad_frame, (0.026, 0.0, 0.0), (0.052, 0.040, 0.020))
    _add_box(pad_frame, (0.056, 0.0, 0.0), (0.030, 0.142, 0.028))
    pad.visual(
        mesh_from_geometry(pad_frame, "pad_tip_carrier"),
        material=gunmetal,
        name="carrier_frame",
    )
    pad.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=_y_cylinder_origin(0.0, 0.0),
        material=brushed_steel,
        name="proximal_barrel",
    )
    pad.visual(
        Box((0.062, 0.155, 0.045)),
        origin=Origin(xyz=(0.099, 0.0, -0.003)),
        material=black_rubber,
        name="rubber_pad",
    )
    for i, z in enumerate((-0.016, 0.0, 0.016)):
        pad.visual(
            Box((0.004, 0.132, 0.004)),
            origin=Origin(xyz=(0.132, 0.0, z - 0.003)),
            material=rubber_rib,
            name=f"pad_rib_{i}",
        )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=middle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "middle_joint",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(middle_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.2, lower=0.0, upper=1.30),
    )
    model.articulation(
        "distal_joint",
        ArticulationType.REVOLUTE,
        parent=distal,
        child=pad,
        origin=Origin(xyz=(distal_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_knuckle")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")
    pad = object_model.get_part("pad_tip")
    root_joint = object_model.get_articulation("root_joint")
    middle_joint = object_model.get_articulation("middle_joint")
    distal_joint = object_model.get_articulation("distal_joint")

    joints = [root_joint, middle_joint, distal_joint]
    ctx.check(
        "three serial revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint types={[j.articulation_type for j in joints]}",
    )
    ctx.check(
        "parallel joint axes bend in one plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_overlap(
        middle,
        root,
        axes="xz",
        elem_a="proximal_barrel",
        elem_b="root_barrel_0",
        min_overlap=0.045,
        name="root joint barrels are coaxial in the bend plane",
    )
    ctx.expect_overlap(
        distal,
        middle,
        axes="xz",
        elem_a="proximal_barrel",
        elem_b="distal_barrel_0",
        min_overlap=0.038,
        name="middle joint barrels are coaxial in the bend plane",
    )
    ctx.expect_overlap(
        pad,
        distal,
        axes="xz",
        elem_a="proximal_barrel",
        elem_b="distal_barrel_0",
        min_overlap=0.034,
        name="distal joint barrels are coaxial in the bend plane",
    )

    middle_aabb = ctx.part_world_aabb(middle)
    distal_aabb = ctx.part_world_aabb(distal)
    pad_aabb = ctx.part_world_aabb(pad)
    if middle_aabb and distal_aabb and pad_aabb:
        middle_x = middle_aabb[1][0] - middle_aabb[0][0]
        distal_x = distal_aabb[1][0] - distal_aabb[0][0]
        distal_y = distal_aabb[1][1] - distal_aabb[0][1]
        pad_y = pad_aabb[1][1] - pad_aabb[0][1]
        ctx.check(
            "middle link is longer than short distal link",
            middle_x > distal_x * 1.35,
            details=f"middle_x={middle_x:.3f}, distal_x={distal_x:.3f}",
        )
        ctx.check(
            "pad tip is broader than distal link",
            pad_y > distal_y * 1.15,
            details=f"pad_y={pad_y:.3f}, distal_y={distal_y:.3f}",
        )
    else:
        ctx.fail("proportion aabbs available", "missing part aabb")

    rest_tip = ctx.part_world_position(pad)
    curled_tip = None
    with ctx.pose({root_joint: 0.70, middle_joint: 0.78, distal_joint: 0.55}):
        curled_tip = ctx.part_world_position(pad)
    ctx.check(
        "positive joint motion curls the finger in the xz plane",
        rest_tip is not None
        and curled_tip is not None
        and curled_tip[2] < rest_tip[2] - 0.09
        and curled_tip[0] < rest_tip[0] - 0.05
        and abs(curled_tip[1] - rest_tip[1]) < 0.002,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
    )

    return ctx.report()


object_model = build_object_model()
