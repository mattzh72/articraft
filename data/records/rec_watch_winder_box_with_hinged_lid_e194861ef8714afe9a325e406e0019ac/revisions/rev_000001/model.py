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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_watch_winder_box")

    leather = model.material("leather", rgba=(0.14, 0.12, 0.11, 1.0))
    suede = model.material("suede", rgba=(0.22, 0.18, 0.15, 1.0))
    charcoal_metal = model.material("charcoal_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    acrylic = model.material("acrylic", rgba=(0.58, 0.68, 0.74, 0.32))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.70, 0.66, 0.56, 1.0))

    outer_depth = 0.160
    outer_width = 0.120
    base_height = 0.082
    floor_thickness = 0.010
    wall_thickness = 0.009

    support_plate_thickness = 0.006
    support_plate_half_span = 0.043
    cradle_axis_z = 0.048

    support_outer_profile = _shift_profile(
        rounded_rect_profile(0.028, 0.058, 0.005, corner_segments=6),
        dy=0.029,
    )
    support_hole_profile = _circle_profile(0.0056, cy=0.038, segments=32)
    support_plate_mesh = _mesh(
        "watch_winder_bearing_plate",
        ExtrudeWithHolesGeometry(
            support_outer_profile,
            [support_hole_profile],
            height=support_plate_thickness,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )

    cushion_mesh = _mesh(
        "watch_winder_cushion",
        ExtrudeGeometry(
            rounded_rect_profile(0.050, 0.060, 0.010, corner_segments=8),
            0.044,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )

    base = model.part("base")
    base.visual(
        Box((outer_depth, outer_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=leather,
        name="floor_tray",
    )
    base.visual(
        Box((outer_depth, wall_thickness, base_height)),
        origin=Origin(
            xyz=(0.0, (outer_width * 0.5) - (wall_thickness * 0.5), base_height * 0.5)
        ),
        material=leather,
        name="left_wall",
    )
    base.visual(
        Box((outer_depth, wall_thickness, base_height)),
        origin=Origin(
            xyz=(0.0, -(outer_width * 0.5) + (wall_thickness * 0.5), base_height * 0.5)
        ),
        material=leather,
        name="right_wall",
    )
    base.visual(
        Box((wall_thickness, outer_width - (2.0 * wall_thickness), base_height)),
        origin=Origin(
            xyz=(-(outer_depth * 0.5) + (wall_thickness * 0.5), 0.0, base_height * 0.5)
        ),
        material=leather,
        name="rear_wall",
    )
    base.visual(
        Box((wall_thickness, outer_width - (2.0 * wall_thickness), base_height)),
        origin=Origin(
            xyz=((outer_depth * 0.5) - (wall_thickness * 0.5), 0.0, base_height * 0.5)
        ),
        material=leather,
        name="front_wall",
    )
    base.visual(
        Box((outer_depth - (2.0 * wall_thickness), outer_width - (2.0 * wall_thickness), 0.003)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.0015)),
        material=suede,
        name="felt_liner",
    )
    base.visual(
        Box((0.028, 0.060, 0.030)),
        origin=Origin(xyz=(-0.052, 0.0, 0.025)),
        material=charcoal_metal,
        name="motor_pod",
    )
    base.visual(
        support_plate_mesh,
        origin=Origin(xyz=(0.0, support_plate_half_span, floor_thickness), rpy=(0.0, 0.0, math.pi)),
        material=satin_steel,
        name="left_bearing_plate",
    )
    base.visual(
        support_plate_mesh,
        origin=Origin(xyz=(0.0, -support_plate_half_span, floor_thickness)),
        material=satin_steel,
        name="right_bearing_plate",
    )
    base.visual(
        Box((0.010, 0.022, 0.026)),
        origin=Origin(xyz=(-0.075, 0.036, 0.088)),
        material=charcoal_metal,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.010, 0.022, 0.026)),
        origin=Origin(xyz=(-0.075, -0.036, 0.088)),
        material=charcoal_metal,
        name="right_hinge_cheek",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.080, 0.036, base_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.080, -0.036, base_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="right_hinge_barrel",
    )
    base.inertial = Inertial.from_geometry(
        Box((outer_depth, outer_width, 0.102)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.160, 0.122, 0.006)),
        origin=Origin(xyz=(0.080, 0.0, 0.043)),
        material=leather,
        name="top_panel",
    )
    lid.visual(
        Box((0.104, 0.072, 0.002)),
        origin=Origin(xyz=(0.088, 0.0, 0.047)),
        material=acrylic,
        name="view_window",
    )
    lid.visual(
        Box((0.160, 0.006, 0.040)),
        origin=Origin(xyz=(0.080, 0.064, 0.020)),
        material=leather,
        name="outer_left_skirt",
    )
    lid.visual(
        Box((0.160, 0.006, 0.040)),
        origin=Origin(xyz=(0.080, -0.064, 0.020)),
        material=leather,
        name="outer_right_skirt",
    )
    lid.visual(
        Box((0.006, 0.128, 0.040)),
        origin=Origin(xyz=(0.157, 0.0, 0.020)),
        material=leather,
        name="outer_front_skirt",
    )
    lid.visual(
        Box((0.010, 0.092, 0.040)),
        origin=Origin(xyz=(0.155, 0.0, 0.020)),
        material=suede,
        name="inner_front_wall",
    )
    lid.visual(
        Box((0.004, 0.044, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=leather,
        name="rear_web",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="center_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.128, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.082, 0.0, 0.025)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0048, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_metal,
        name="axle_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, support_plate_half_span + 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="left_collar",
    )
    cradle.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -(support_plate_half_span + 0.005), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="right_collar",
    )
    cradle.visual(
        Box((0.056, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
        material=charcoal_metal,
        name="rotor_plate",
    )
    cradle.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=cushion_fabric,
        name="watch_cushion",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.060, 0.100, 0.074)),
        mass=0.24,
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.080, 0.0, base_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, cradle_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

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
        "lid hinge axis opens upward",
        lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"expected (0.0, -1.0, 0.0), got {lid_hinge.axis}",
    )
    ctx.check(
        "cradle axis runs left-right",
        cradle_spin.axis == (0.0, 1.0, 0.0),
        details=f"expected (0.0, 1.0, 0.0), got {cradle_spin.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            base,
            elem_a="inner_front_wall",
            elem_b="front_wall",
            name="lid closes onto front stop",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="left_collar",
            elem_b="left_bearing_plate",
            name="left cradle support seats on bearing plate",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="right_collar",
            elem_b="right_bearing_plate",
            name="right cradle support seats on bearing plate",
        )
        ctx.expect_gap(
            lid,
            cradle,
            axis="z",
            positive_elem="top_panel",
            negative_elem="watch_cushion",
            min_gap=0.035,
            name="closed lid clears cushion height",
        )
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            positive_elem="watch_cushion",
            negative_elem="floor_tray",
            min_gap=0.0075,
            name="cushion clears floor tray",
        )

    with ctx.pose({lid_hinge: 1.10}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="outer_front_skirt",
            negative_elem="rear_wall",
            min_gap=0.080,
            name="opened lid lifts its free edge above box",
        )

    with ctx.pose({cradle_spin: math.pi / 2.0}):
        ctx.expect_gap(
            base,
            cradle,
            axis="x",
            positive_elem="front_wall",
            negative_elem="watch_cushion",
            min_gap=0.030,
            name="spun cradle keeps front clearance",
        )
        ctx.expect_gap(
            cradle,
            base,
            axis="x",
            positive_elem="watch_cushion",
            negative_elem="rear_wall",
            min_gap=0.030,
            name="spun cradle keeps rear clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
