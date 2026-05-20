from __future__ import annotations

import math

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


def _fork_tine_mesh(length: float, width: float, height: float, tip_height: float, taper: float, name: str):
    """A single tapered fork tine with a flat underside and sloped nose."""
    geom = MeshGeometry()
    stations = ((0.0, height), (length - taper, height), (length, tip_height))
    half_w = width / 2.0
    rings = []
    for x, top_z in stations:
        rings.append(
            (
                geom.add_vertex(x, -half_w, 0.0),
                geom.add_vertex(x, half_w, 0.0),
                geom.add_vertex(x, half_w, top_z),
                geom.add_vertex(x, -half_w, top_z),
            )
        )

    for a, b in zip(rings[:-1], rings[1:]):
        # bottom
        geom.add_face(a[0], b[0], b[1])
        geom.add_face(a[0], b[1], a[1])
        # top
        geom.add_face(a[3], a[2], b[2])
        geom.add_face(a[3], b[2], b[3])
        # left side
        geom.add_face(a[0], a[3], b[3])
        geom.add_face(a[0], b[3], b[0])
        # right side
        geom.add_face(a[1], b[1], b[2])
        geom.add_face(a[1], b[2], a[2])

    rear = rings[0]
    front = rings[-1]
    geom.add_face(rear[0], rear[1], rear[2])
    geom.add_face(rear[0], rear[2], rear[3])
    geom.add_face(front[0], front[2], front[1])
    geom.add_face(front[0], front[3], front[2])
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_frame_fork_mast")

    frame_orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    fork_steel = model.material("worn_fork_steel", rgba=(0.33, 0.34, 0.33, 1.0))
    black_rubber = model.material("black_guide_roller", rgba=(0.02, 0.02, 0.018, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.10, 0.10, 1.80)),
        origin=Origin(xyz=(0.0, 0.45, 0.90)),
        material=frame_orange,
        name="side_rail_0",
    )
    rear_frame.visual(
        Box((0.10, 0.10, 1.80)),
        origin=Origin(xyz=(0.0, -0.45, 0.90)),
        material=frame_orange,
        name="side_rail_1",
    )
    rear_frame.visual(
        Box((0.12, 1.00, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.75)),
        material=frame_orange,
        name="top_crossbar",
    )
    rear_frame.visual(
        Box((0.12, 1.00, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=frame_orange,
        name="bottom_crossbar",
    )
    rear_frame.visual(
        Box((0.36, 0.13, 0.055)),
        origin=Origin(xyz=(-0.14, 0.45, 0.0275)),
        material=dark_steel,
        name="base_shoe_0",
    )
    rear_frame.visual(
        Box((0.36, 0.13, 0.055)),
        origin=Origin(xyz=(-0.14, -0.45, 0.0275)),
        material=dark_steel,
        name="base_shoe_1",
    )
    rear_frame.visual(
        Box((0.030, 0.030, 1.50)),
        origin=Origin(xyz=(0.055, 0.385, 0.90)),
        material=dark_steel,
        name="guide_track_0",
    )
    rear_frame.visual(
        Box((0.030, 0.030, 1.50)),
        origin=Origin(xyz=(0.055, -0.385, 0.90)),
        material=dark_steel,
        name="guide_track_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.060, 0.68, 0.49)),
        origin=Origin(xyz=(0.035, 0.0, 0.245)),
        material=dark_steel,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.070, 0.78, 0.070)),
        origin=Origin(xyz=(0.070, 0.0, 0.455)),
        material=dark_steel,
        name="upper_backrest_bar",
    )
    carriage.visual(
        Box((0.050, 0.74, 0.055)),
        origin=Origin(xyz=(0.078, 0.0, 0.300)),
        material=dark_steel,
        name="middle_backrest_bar",
    )
    for idx, y in enumerate((-0.28, 0.28)):
        carriage.visual(
            Box((0.045, 0.050, 0.43)),
            origin=Origin(xyz=(0.080, y, 0.235)),
            material=dark_steel,
            name=f"backrest_upright_{idx}",
        )
    for idx, y in enumerate((-0.22, 0.22)):
        carriage.visual(
            Box((0.080, 0.13, 0.42)),
            origin=Origin(xyz=(0.080, y, 0.210)),
            material=fork_steel,
            name=f"fork_heel_{idx}",
        )
        carriage.visual(
            _fork_tine_mesh(0.88, 0.105, 0.060, 0.022, 0.20, f"tine_mesh_{idx}"),
            origin=Origin(xyz=(0.045, y, 0.0)),
            material=fork_steel,
            name=f"fork_tine_{idx}",
        )
    for idx, (y, z) in enumerate(((0.355, 0.125), (-0.355, 0.125), (0.355, 0.365), (-0.355, 0.365))):
        carriage.visual(
            Box((0.050, 0.030, 0.14)),
            origin=Origin(xyz=(-0.005, y, z)),
            material=black_rubber,
            name=f"guide_shoe_{idx}",
        )
    for idx, (y, z) in enumerate(((0.348, 0.125), (-0.348, 0.125), (0.348, 0.365), (-0.348, 0.365))):
        carriage.visual(
            Cylinder(radius=0.024, length=0.034),
            origin=Origin(xyz=(0.085, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"guide_roller_{idx}",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=carriage,
        origin=Origin(xyz=(0.080, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    ctx.expect_gap(
        carriage,
        rear_frame,
        axis="x",
        positive_elem="carriage_plate",
        negative_elem="guide_track_0",
        min_gap=0.004,
        max_gap=0.040,
        name="carriage plate rides just in front of mast guide tracks",
    )
    ctx.expect_contact(
        carriage,
        rear_frame,
        elem_a="guide_shoe_0",
        elem_b="guide_track_0",
        contact_tol=0.001,
        name="guide shoe bears on mast track",
    )
    ctx.expect_within(
        carriage,
        rear_frame,
        axes="y",
        inner_elem="carriage_plate",
        outer_elem="top_crossbar",
        margin=0.0,
        name="carriage plate fits between side rails",
    )
    ctx.expect_overlap(
        carriage,
        rear_frame,
        axes="z",
        elem_a="carriage_plate",
        elem_b="guide_track_0",
        min_overlap=0.20,
        name="low carriage stays engaged with vertical guide tracks",
    )

    plate_box = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    tine_box = ctx.part_element_world_aabb(carriage, elem="fork_tine_0")
    ctx.check(
        "fork tines project forward from carriage",
        plate_box is not None and tine_box is not None and tine_box[1][0] > plate_box[1][0] + 0.70,
        details=f"plate_aabb={plate_box}, tine_aabb={tine_box}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.75}):
        ctx.expect_overlap(
            carriage,
            rear_frame,
            axes="z",
            elem_a="carriage_plate",
            elem_b="guide_track_0",
            min_overlap=0.20,
            name="raised carriage remains captured on guide tracks",
        )
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "single prismatic joint lifts carriage and forks together",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
