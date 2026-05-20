from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _extruded_xz_profile(
    profile: list[tuple[float, float]],
    *,
    y_min: float,
    y_max: float,
) -> MeshGeometry:
    """Extrude an X/Z cross-section along Y for the cast top cap."""
    mesh = MeshGeometry()
    front = [mesh.add_vertex(x, y_min, z) for x, z in profile]
    back = [mesh.add_vertex(x, y_max, z) for x, z in profile]
    count = len(profile)
    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(mesh, front[index], front[nxt], back[nxt], back[index])

    # Triangulate the end caps with a fan. The profile is convex enough for this
    # simple construction and gives a single closed, cast-looking piece.
    for index in range(1, count - 1):
        mesh.add_face(front[0], front[index], front[index + 1])
        mesh.add_face(back[0], back[index + 1], back[index])
    return mesh


def _build_cast_cap_mesh() -> MeshGeometry:
    half_width = 0.0525
    z_base = 0.058
    z_top = 0.107
    points: list[tuple[float, float]] = [(-half_width, z_base), (half_width, z_base)]
    # Broad half-ellipse top: reminiscent of a cast school sharpener housing.
    for step in range(11):
        t = step / 10.0
        angle = t * math.pi
        x = half_width * math.cos(angle)
        z = z_base + (z_top - z_base) * math.sin(angle)
        points.append((x, z))
    return _extruded_xz_profile(points, y_min=-0.090, y_max=0.090)


def _material(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_crank_pencil_sharpener")

    cast_green = _material(model, "cast_green", (0.40, 0.51, 0.43, 1.0))
    darker_cast = _material(model, "dark_cast_edges", (0.25, 0.31, 0.27, 1.0))
    drawer_plastic = _material(model, "smoked_drawer", (0.18, 0.21, 0.22, 0.86))
    drawer_shadow = _material(model, "drawer_shadow", (0.05, 0.055, 0.055, 1.0))
    dark_metal = _material(model, "dark_metal", (0.10, 0.105, 0.11, 1.0))
    bright_metal = _material(model, "worn_steel", (0.64, 0.66, 0.67, 1.0))
    black_rubber = _material(model, "black_rubber", (0.025, 0.025, 0.023, 1.0))
    warm_knob = _material(model, "brown_bakelite", (0.31, 0.17, 0.075, 1.0))

    # Root cast housing, kept open through the lower front so the waste drawer
    # can slide inside without using a solid-body overlap proxy.
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_cast_cap_mesh(), "cast_housing_cap"),
        material=cast_green,
        name="arched_cast_cap",
    )
    body.visual(
        Box((0.108, 0.184, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=darker_cast,
        name="base_plate",
    )
    body.visual(
        Box((0.016, 0.184, 0.064)),
        origin=Origin(xyz=(-0.046, 0.0, 0.040)),
        material=cast_green,
        name="side_wall_0",
    )
    body.visual(
        Box((0.016, 0.184, 0.064)),
        origin=Origin(xyz=(0.046, 0.0, 0.040)),
        material=cast_green,
        name="side_wall_1",
    )
    body.visual(
        Box((0.108, 0.012, 0.058)),
        origin=Origin(xyz=(0.0, 0.084, 0.041)),
        material=cast_green,
        name="rear_wall",
    )
    body.visual(
        Box((0.108, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.084, 0.079)),
        material=cast_green,
        name="front_upper_casting",
    )
    body.visual(
        Box((0.012, 0.012, 0.046)),
        origin=Origin(xyz=(-0.044, -0.084, 0.039)),
        material=cast_green,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.012, 0.012, 0.046)),
        origin=Origin(xyz=(0.044, -0.084, 0.039)),
        material=cast_green,
        name="front_jamb_1",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, -0.093, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="pencil_bezel",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, -0.098, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drawer_shadow,
        name="pencil_throat",
    )
    body.visual(
        Box((0.035, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.101, 0.084)),
        material=drawer_shadow,
        name="pencil_grip_slit",
    )
    # Small screw bosses on the same front casting, embedded just enough to read
    # as seated rather than separate floating dots.
    for index, x in enumerate((-0.032, 0.032)):
        body.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, -0.092, 0.096), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_metal,
            name=f"front_screw_{index}",
        )
    body.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.060, 0.014, 0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_cast,
        name="crank_bearing",
    )
    body.visual(
        Cylinder(radius=0.063, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=black_rubber,
        name="suction_cup",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, -0.020, 0.004)),
        material=black_rubber,
        name="suction_stem",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.060, -0.026, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="lock_pivot_pin",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.074, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=drawer_plastic,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.066, 0.150, 0.004)),
        origin=Origin(xyz=(0.0, 0.075, -0.018)),
        material=drawer_plastic,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.004, 0.145, 0.026)),
        origin=Origin(xyz=(-0.035, 0.078, -0.006)),
        material=drawer_plastic,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.004, 0.145, 0.026)),
        origin=Origin(xyz=(0.035, 0.078, -0.006)),
        material=drawer_plastic,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.066, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, 0.151, -0.006)),
        material=drawer_plastic,
        name="tray_back",
    )
    drawer.visual(
        Box((0.046, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.011, 0.004)),
        material=drawer_shadow,
        name="finger_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.013),
        origin=Origin(xyz=(0.0065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="crank_hub",
    )
    crank.visual(
        Box((0.012, 0.014, 0.072)),
        origin=Origin(xyz=(0.010, 0.0, -0.026)),
        material=bright_metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.021, 0.0, -0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="end_lug",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(0.038, 0.0, -0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="knob_pin",
    )

    spinner_knob = model.part("spinner_knob")
    spinner_knob.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_knob,
        name="knob_grip",
    )
    spinner_knob.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.0355, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="knob_end_cap",
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lever_hub",
    )
    lock_lever.visual(
        Box((0.010, 0.046, 0.009)),
        origin=Origin(xyz=(-0.008, -0.026, -0.004)),
        material=dark_metal,
        name="lever_blade",
    )
    lock_lever.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(-0.008, -0.051, -0.004)),
        material=black_rubber,
        name="lever_thumb_pad",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.089, 0.039)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.075),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.070, 0.014, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=9.0),
    )
    model.articulation(
        "crank_to_spinner_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=spinner_knob,
        origin=Origin(xyz=(0.027, 0.0, -0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=18.0),
    )
    model.articulation(
        "body_to_lock_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_lever,
        origin=Origin(xyz=(-0.066, -0.026, 0.011)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-0.55, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    spinner_knob = object_model.get_part("spinner_knob")
    lock_lever = object_model.get_part("lock_lever")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    knob_joint = object_model.get_articulation("crank_to_spinner_knob")
    lever_joint = object_model.get_articulation("body_to_lock_lever")

    ctx.check(
        "crank rotates continuously",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_joint.articulation_type}",
    )
    ctx.check(
        "spinner knob rotates continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "drawer is prismatic",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_joint.articulation_type}",
    )
    ctx.check(
        "lock lever is limited revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_joint.motion_limits is not None
        and lever_joint.motion_limits.lower < 0.0
        and lever_joint.motion_limits.upper > 0.0,
        details=f"type={lever_joint.articulation_type}, limits={lever_joint.motion_limits}",
    )

    # The wooden spinner knob has a real captured axle.  The tiny steel pin is
    # intentionally inside the knob cylinder rather than floating beside it.
    ctx.allow_overlap(
        crank,
        spinner_knob,
        elem_a="knob_pin",
        elem_b="knob_grip",
        reason="The handle knob is captured by a steel axle running through its bore.",
    )
    ctx.expect_within(
        crank,
        spinner_knob,
        axes="yz",
        inner_elem="knob_pin",
        outer_elem="knob_grip",
        margin=0.001,
        name="knob pin stays centered in spinner grip",
    )
    ctx.expect_overlap(
        crank,
        spinner_knob,
        axes="x",
        elem_a="knob_pin",
        elem_b="knob_grip",
        min_overlap=0.020,
        name="spinner grip remains captured on the pin",
    )

    ctx.expect_gap(
        crank,
        body,
        axis="x",
        positive_elem="crank_hub",
        negative_elem="crank_bearing",
        max_gap=0.004,
        max_penetration=0.0,
        name="crank hub seats just outside bearing",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="x",
        inner_elem="tray_bottom",
        outer_elem="base_plate",
        margin=0.001,
        name="drawer tray fits between side walls",
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="z",
        positive_elem="tray_bottom",
        negative_elem="base_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="drawer tray rides above cast base",
    )

    closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.075}):
        open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray_bottom",
            elem_b="base_plate",
            min_overlap=0.055,
            name="open drawer still retained in housing",
        )
    ctx.check(
        "drawer pulls forward along depth",
        closed_pos is not None and open_pos is not None and open_pos[1] < closed_pos[1] - 0.070,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
