from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _loft_loops(loops: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    """Triangulate equal-sized cross-section loops into a closed mesh."""
    geom = MeshGeometry()
    count = len(loops[0])
    for loop in loops:
        for x, y, z in loop:
            geom.add_vertex(x, y, z)
    for section in range(len(loops) - 1):
        a0 = section * count
        b0 = (section + 1) * count
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(a0 + i, b0 + i, b0 + j)
            geom.add_face(a0 + i, b0 + j, a0 + j)
    for section_index, loop in ((0, loops[0]), (len(loops) - 1, loops[-1])):
        cx = sum(p[0] for p in loop) / count
        cy = sum(p[1] for p in loop) / count
        cz = sum(p[2] for p in loop) / count
        center_index = geom.add_vertex(cx, cy, cz)
        offset = section_index * count
        for i in range(count):
            j = (i + 1) % count
            if section_index == 0:
                geom.add_face(center_index, offset + j, offset + i)
            else:
                geom.add_face(center_index, offset + i, offset + j)
    return geom


def _arched_loop(
    x: float,
    width: float,
    bottom_z: float,
    height: float,
    *,
    y_center: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    """Flat-bottom, rounded-top section loop in an X-constant plane."""
    points: list[tuple[float, float, float]] = []
    for i in range(segments + 1):
        theta = pi - (pi * i / segments)
        y = y_center + 0.5 * width * cos(theta)
        z = bottom_z + height * sin(theta)
        points.append((x, y, z))
    return points


def _arched_loft(
    sections: list[tuple[float, float, float, float]],
    *,
    y_center: float = 0.0,
    segments: int = 24,
) -> MeshGeometry:
    return _loft_loops(
        [_arched_loop(x, width, bottom_z, height, y_center=y_center, segments=segments) for x, width, bottom_z, height in sections]
    )


def _add_wheel_visuals(part, prefix: str, rubber: Material, silver: Material, dark: Material) -> None:
    """Add a broad toy racing tire and detailed rim, both spinning about local X."""
    tire = TireGeometry(
        0.043,
        0.036,
        inner_radius=0.028,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.0035, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0018),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.003, radius=0.002),
    )
    rim = WheelGeometry(
        0.029,
        0.031,
        rim=WheelRim(
            inner_radius=0.019,
            flange_height=0.003,
            flange_thickness=0.002,
            bead_seat_depth=0.0015,
        ),
        hub=WheelHub(
            radius=0.010,
            width=0.026,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.015, hole_diameter=0.002),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0018, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.014),
    )
    part.visual(_mesh(tire, f"{prefix}_tire"), material=rubber, name="tire")
    part.visual(_mesh(rim, f"{prefix}_rim"), material=silver, name="rim")
    part.visual(
        Cylinder(radius=0.009, length=0.033),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="hub_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_race_car")

    red = model.material("gloss_red", rgba=(0.82, 0.04, 0.025, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.025, 1.0))
    dark = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    silver = model.material("brushed_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    glass = model.material("smoke_blue_glass", rgba=(0.10, 0.23, 0.42, 0.48))

    body = model.part("body")
    body_shell = _arched_loft(
        [
            (0.172, 0.030, 0.041, 0.016),
            (0.128, 0.082, 0.043, 0.031),
            (0.060, 0.122, 0.046, 0.044),
            (-0.015, 0.138, 0.049, 0.055),
            (-0.095, 0.136, 0.047, 0.058),
            (-0.168, 0.112, 0.046, 0.058),
        ],
        segments=32,
    )
    body.visual(_mesh(body_shell, "wedge_body"), material=red, name="body_shell")
    # Two metal axles are carried by the body; wheel parts spin on the same lines.
    for axle_x, axle_name in ((0.105, "front_axle"), (-0.118, "rear_axle")):
        body.visual(
            Cylinder(radius=0.006, length=0.200),
            origin=Origin(xyz=(axle_x, 0.0, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=axle_name,
        )
    # Visible hinge hardware embedded into the shell for the side canopy and rear cover.
    body.visual(
        Cylinder(radius=0.0035, length=0.092),
        origin=Origin(xyz=(-0.010, -0.044, 0.101), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="canopy_hinge_rail",
    )
    body.visual(
        Box((0.092, 0.006, 0.010)),
        origin=Origin(xyz=(-0.010, -0.044, 0.094)),
        material=dark,
        name="canopy_hinge_boss",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.105),
        origin=Origin(xyz=(-0.168, 0.0, 0.104), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="engine_hinge_rail",
    )

    canopy = model.part("canopy")
    canopy_bubble = _arched_loft(
        [
            (-0.044, 0.052, 0.000, 0.020),
            (-0.020, 0.068, 0.000, 0.034),
            (0.018, 0.070, 0.000, 0.036),
            (0.046, 0.054, 0.000, 0.024),
        ],
        y_center=0.035,
        segments=28,
    )
    canopy.visual(_mesh(canopy_bubble, "canopy_bubble"), material=glass, name="canopy_bubble")
    canopy.visual(
        Box((0.092, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.001, 0.001)),
        material=dark,
        name="hinge_leaf",
    )

    engine_cover = model.part("engine_cover")
    cover_panel = _arched_loft(
        [
            (0.000, 0.094, 0.000, 0.012),
            (0.036, 0.104, 0.000, 0.016),
            (0.078, 0.100, 0.000, 0.014),
            (0.108, 0.084, 0.000, 0.012),
        ],
        segments=28,
    )
    engine_cover.visual(_mesh(cover_panel, "engine_cover_panel"), material=red, name="cover_panel")
    for index, x_pos in enumerate((0.028, 0.043, 0.058, 0.073)):
        engine_cover.visual(
            Box((0.008, 0.068, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.014)),
            material=black,
            name=f"louver_{index}",
        )
    engine_cover.visual(
        Box((0.014, 0.100, 0.010)),
        origin=Origin(xyz=(-0.003, 0.0, 0.000)),
        material=dark,
        name="rear_hinge_leaf",
    )

    wheel_specs = {
        "front_left_wheel": (0.105, 0.092, 0.045),
        "front_right_wheel": (0.105, -0.092, 0.045),
        "rear_left_wheel": (-0.118, 0.092, 0.045),
        "rear_right_wheel": (-0.118, -0.092, 0.045),
    }
    for wheel_name in wheel_specs:
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, wheel_name, rubber=black, silver=silver, dark=dark)

    for wheel_name, (x_pos, y_pos, z_pos) in wheel_specs.items():
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_name,
            origin=Origin(xyz=(x_pos, y_pos, z_pos), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=35.0),
        )

    model.articulation(
        "body_to_canopy",
        ArticulationType.REVOLUTE,
        parent=body,
        child=canopy,
        origin=Origin(xyz=(-0.010, -0.040, 0.107)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_engine_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=engine_cover,
        origin=Origin(xyz=(-0.170, 0.0, 0.109)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    canopy = object_model.get_part("canopy")
    cover = object_model.get_part("engine_cover")
    canopy_joint = object_model.get_articulation("body_to_canopy")
    cover_joint = object_model.get_articulation("body_to_engine_cover")

    wheel_joints = [
        object_model.get_articulation("body_to_front_left_wheel"),
        object_model.get_articulation("body_to_front_right_wheel"),
        object_model.get_articulation("body_to_rear_left_wheel"),
        object_model.get_articulation("body_to_rear_right_wheel"),
    ]
    wheel_axles = {
        "front_left_wheel": "front_axle",
        "front_right_wheel": "front_axle",
        "rear_left_wheel": "rear_axle",
        "rear_right_wheel": "rear_axle",
    }
    for wheel_name, axle_elem in wheel_axles.items():
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=axle_elem,
            elem_b="hub_sleeve",
            reason="The toy car axle is intentionally captured inside the rotating wheel hub sleeve.",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="xyz",
            min_overlap=0.008,
            elem_a="hub_sleeve",
            elem_b=axle_elem,
            name=f"{wheel_name} hub sleeve is retained on its axle",
        )
    ctx.allow_overlap(
        body,
        canopy,
        elem_a="canopy_hinge_rail",
        elem_b="hinge_leaf",
        reason="The canopy hinge leaf wraps around the fixed side hinge rail.",
    )
    ctx.expect_overlap(
        canopy,
        body,
        axes="x",
        min_overlap=0.060,
        elem_a="hinge_leaf",
        elem_b="canopy_hinge_rail",
        name="canopy side hinge spans the cockpit",
    )
    ctx.allow_overlap(
        body,
        cover,
        elem_a="engine_hinge_rail",
        elem_b="rear_hinge_leaf",
        reason="The rear engine cover hinge leaf is captured around the rear hinge rail.",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="y",
        min_overlap=0.080,
        elem_a="rear_hinge_leaf",
        elem_b="engine_hinge_rail",
        name="engine cover hinge spans the rear deck",
    )
    ctx.check(
        "four continuous wheel axles",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_joints),
        details=str([joint.articulation_type for joint in wheel_joints]),
    )
    ctx.check(
        "canopy and engine cover use upward hinges",
        canopy_joint.articulation_type == ArticulationType.REVOLUTE
        and cover_joint.articulation_type == ArticulationType.REVOLUTE
        and canopy_joint.motion_limits is not None
        and cover_joint.motion_limits is not None
        and canopy_joint.motion_limits.upper > 1.0
        and cover_joint.motion_limits.upper > 0.9,
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.060,
        elem_a="cover_panel",
        elem_b="body_shell",
        name="rear cover sits over the engine deck",
    )
    ctx.expect_overlap(
        canopy,
        body,
        axes="xy",
        min_overlap=0.040,
        elem_a="canopy_bubble",
        elem_b="body_shell",
        name="central canopy sits on the cockpit opening",
    )

    closed_canopy_aabb = ctx.part_element_world_aabb(canopy, elem="canopy_bubble")
    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({canopy_joint: 1.0, cover_joint: 0.9}):
        open_canopy_aabb = ctx.part_element_world_aabb(canopy, elem="canopy_bubble")
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "canopy opens upward from side hinge",
        closed_canopy_aabb is not None
        and open_canopy_aabb is not None
        and open_canopy_aabb[1][2] > closed_canopy_aabb[1][2] + 0.015,
        details=f"closed={closed_canopy_aabb}, open={open_canopy_aabb}",
    )
    ctx.check(
        "engine cover opens upward from rear hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.020,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
