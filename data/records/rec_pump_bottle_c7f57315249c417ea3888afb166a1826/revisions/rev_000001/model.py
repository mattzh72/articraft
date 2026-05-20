from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _faceted_loop(
    *,
    width: float,
    depth: float,
    chamfer: float,
    z: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    chamfer = min(chamfer, half_width * 0.45, half_depth * 0.45)
    return [
        (-half_width + chamfer, -half_depth, z),
        (half_width - chamfer, -half_depth, z),
        (half_width, -half_depth + chamfer, z),
        (half_width, half_depth - chamfer, z),
        (half_width - chamfer, half_depth, z),
        (-half_width + chamfer, half_depth, z),
        (-half_width, half_depth - chamfer, z),
        (-half_width, -half_depth + chamfer, z),
    ]


def _bridge_loops(
    geom: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
    *,
    outward: bool,
) -> None:
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        if outward:
            _add_quad(
                geom,
                loop_a[index],
                loop_a[next_index],
                loop_b[next_index],
                loop_b[index],
            )
        else:
            _add_quad(
                geom,
                loop_a[index],
                loop_b[index],
                loop_b[next_index],
                loop_a[next_index],
            )


def _rounded_plate_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    corner_radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, corner_radius, corner_segments=8),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def _build_bottle_shell() -> MeshGeometry:
    wall = 0.0022
    sections = [
        (0.000, 0.062, 0.041, 0.010),
        (0.105, 0.062, 0.041, 0.010),
        (0.128, 0.056, 0.037, 0.009),
        (0.145, 0.042, 0.031, 0.006),
        (0.154, 0.024, 0.024, 0.004),
        (0.160, 0.034, 0.034, 0.006),
        (0.170, 0.034, 0.034, 0.006),
    ]

    geom = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []

    for z, width, depth, chamfer in sections:
        outer_ids = [
            geom.add_vertex(*point)
            for point in _faceted_loop(width=width, depth=depth, chamfer=chamfer, z=z)
        ]
        outer_loops.append(outer_ids)

    for index, (z, width, depth, chamfer) in enumerate(sections):
        inner_z = 0.003 if index == 0 else z
        inner_width = width - (2.0 * wall)
        inner_depth = depth - (2.0 * wall)
        inner_chamfer = max(chamfer - wall * 0.7, wall * 0.6)
        inner_ids = [
            geom.add_vertex(*point)
            for point in _faceted_loop(
                width=inner_width,
                depth=inner_depth,
                chamfer=inner_chamfer,
                z=inner_z,
            )
        ]
        inner_loops.append(inner_ids)

    for index in range(len(outer_loops) - 1):
        _bridge_loops(geom, outer_loops[index], outer_loops[index + 1], outward=True)
        _bridge_loops(geom, inner_loops[index], inner_loops[index + 1], outward=False)

    _bridge_loops(geom, outer_loops[-1], inner_loops[-1], outward=True)
    _bridge_loops(geom, inner_loops[0], outer_loops[0], outward=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_pump_bottle")

    frosted_body = model.material("frosted_body", rgba=(0.90, 0.85, 0.80, 0.88))
    white_pump = model.material("white_pump", rgba=(0.96, 0.97, 0.98, 1.0))
    pearl_cap = model.material("pearl_cap", rgba=(0.97, 0.97, 0.98, 0.78))
    shadow = model.material("shadow", rgba=(0.35, 0.36, 0.38, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_shell(), "faceted_bottle_shell"),
        material=frosted_body,
        name="body_shell",
    )
    bottle.visual(
        Box((0.024, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.017, 0.163)),
        material=white_pump,
        name="front_collar_face",
    )
    bottle.visual(
        Box((0.024, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.017, 0.163)),
        material=white_pump,
        name="rear_collar_face",
    )
    bottle.visual(
        Box((0.004, 0.026, 0.014)),
        origin=Origin(xyz=(0.017, 0.0, 0.163)),
        material=white_pump,
        name="right_collar_face",
    )
    bottle.visual(
        Box((0.004, 0.026, 0.014)),
        origin=Origin(xyz=(-0.017, 0.0, 0.163)),
        material=white_pump,
        name="left_collar_face",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.068, 0.048, 0.172)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
    )

    plunger = model.part("pump_plunger")
    plunger.visual(
        Cylinder(radius=0.0098, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=white_pump,
        name="pump_stem",
    )
    plunger.visual(
        _rounded_plate_mesh(
            width=0.040,
            depth=0.022,
            height=0.010,
            corner_radius=0.004,
            name="pump_actuator_top",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=white_pump,
        name="actuator_top",
    )
    plunger.visual(
        Box((0.012, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, 0.022)),
        material=white_pump,
        name="nozzle_body",
    )
    plunger.visual(
        Box((0.004, 0.0012, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0214, 0.022)),
        material=shadow,
        name="nozzle_opening",
    )
    plunger.visual(
        Box((0.022, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.003, 0.029)),
        material=white_pump,
        name="hinge_bridge",
    )
    plunger.visual(
        Cylinder(radius=0.0018, length=0.0078),
        origin=Origin(
            xyz=(-0.0089, 0.003, 0.031),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_pump,
        name="left_hinge_barrel",
    )
    plunger.visual(
        Cylinder(radius=0.0018, length=0.0078),
        origin=Origin(
            xyz=(0.0089, 0.003, 0.031),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_pump,
        name="right_hinge_barrel",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.042, 0.030, 0.086)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.006, -0.004)),
    )

    model.articulation(
        "bottle_to_pump_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )

    cap = model.part("nozzle_cap")
    cap.visual(
        Cylinder(radius=0.0017, length=0.0094),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pearl_cap,
        name="cap_barrel",
    )
    cap.visual(
        Box((0.016, 0.0025, 0.004)),
        origin=Origin(xyz=(0.0, 0.0013, -0.0020)),
        material=pearl_cap,
        name="cap_link",
    )
    cap.visual(
        _rounded_plate_mesh(
            width=0.022,
            depth=0.020,
            height=0.003,
            corner_radius=0.003,
            name="nozzle_cap_roof",
        ),
        origin=Origin(xyz=(0.0, 0.011, -0.0025)),
        material=pearl_cap,
        name="cap_roof",
    )
    cap.visual(
        Box((0.0025, 0.018, 0.008)),
        origin=Origin(xyz=(-0.00975, 0.011, -0.0065)),
        material=pearl_cap,
        name="left_cap_wall",
    )
    cap.visual(
        Box((0.0025, 0.018, 0.008)),
        origin=Origin(xyz=(0.00975, 0.011, -0.0065)),
        material=pearl_cap,
        name="right_cap_wall",
    )
    cap.visual(
        Box((0.022, 0.0025, 0.008)),
        origin=Origin(xyz=(0.0, 0.0205, -0.0065)),
        material=pearl_cap,
        name="front_cap_wall",
    )
    cap.visual(
        Box((0.010, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, 0.0222, -0.0108)),
        material=pearl_cap,
        name="thumb_tab",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.014)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.012, -0.005)),
    )

    model.articulation(
        "pump_plunger_to_nozzle_cap",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=cap,
        origin=Origin(xyz=(0.0, 0.003, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    bottle = object_model.get_part("bottle")
    plunger = object_model.get_part("pump_plunger")
    cap = object_model.get_part("nozzle_cap")
    plunger_slide = object_model.get_articulation("bottle_to_pump_plunger")
    cap_hinge = object_model.get_articulation("pump_plunger_to_nozzle_cap")

    ctx.expect_overlap(
        cap,
        plunger,
        axes="xy",
        elem_a="cap_roof",
        elem_b="nozzle_body",
        min_overlap=0.008,
        name="closed cap covers the nozzle footprint",
    )
    ctx.expect_gap(
        cap,
        plunger,
        axis="z",
        positive_elem="cap_roof",
        negative_elem="nozzle_body",
        min_gap=0.0005,
        max_gap=0.004,
        name="closed cap sits just above the nozzle body",
    )
    ctx.expect_within(
        plunger,
        bottle,
        axes="xy",
        inner_elem="pump_stem",
        outer_elem="body_shell",
        margin=0.0,
        name="pump stem stays inside the bottle footprint",
    )
    ctx.expect_overlap(
        plunger,
        bottle,
        axes="z",
        elem_a="pump_stem",
        elem_b="body_shell",
        min_overlap=0.050,
        name="pump stem remains inserted into the bottle at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: plunger_slide.motion_limits.upper}):
        pressed_pos = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger,
            bottle,
            axes="z",
            elem_a="pump_stem",
            elem_b="body_shell",
            min_overlap=0.055,
            name="pump stem remains inserted when fully pressed",
        )
    ctx.check(
        "pump plunger depresses downward",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.008,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    closed_cap_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({cap_hinge: 1.8}):
        open_cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap swings upward on its hinge",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.012,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
