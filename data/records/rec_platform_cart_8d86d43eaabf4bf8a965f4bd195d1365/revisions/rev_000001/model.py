from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

LEFT_HANDLE_HINGE_LUG = "left_handle_hinge_lug"
RIGHT_HANDLE_HINGE_LUG = "right_handle_hinge_lug"
LEFT_REAR_AXLE_STUB = "left_rear_axle_stub"
RIGHT_REAR_AXLE_STUB = "right_rear_axle_stub"


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_mesh(
    *,
    overall_radius: float,
    tire_tube: float,
    width: float,
    hub_radius: float,
    hub_width: float,
) -> object:
    geom = TorusGeometry(
        radius=overall_radius - tire_tube,
        tube=tire_tube,
        radial_segments=18,
        tubular_segments=40,
    ).rotate_x(pi / 2.0)
    geom.merge(
        CylinderGeometry(radius=hub_radius, height=hub_width)
        .rotate_x(pi / 2.0)
    )
    spoke = BoxGeometry((overall_radius * 1.20, hub_width * 0.62, tire_tube * 0.52))
    geom.merge(spoke.copy())
    geom.merge(spoke.copy().rotate_y(pi / 2.0))
    geom.merge(spoke.copy().rotate_y(pi / 4.0))
    geom.merge(spoke.copy().rotate_y(-pi / 4.0))
    return geom


def _build_caster_support_mesh(*, trail: float, fork_gap: float, fork_drop: float) -> object:
    geom = CylinderGeometry(radius=0.011, height=0.048).translate(0.0, 0.0, -0.027)
    geom.merge(BoxGeometry((0.058, 0.048, 0.006)).translate(0.0, 0.0, -0.003))
    geom.merge(BoxGeometry((0.040, 0.018, 0.014)).translate(-trail * 0.35, 0.0, -0.015))
    geom.merge(BoxGeometry((trail * 0.65, fork_gap + 0.012, 0.010)).translate(-trail * 0.70, 0.0, -0.019))
    plate = BoxGeometry((0.016, 0.006, fork_drop))
    geom.merge(plate.copy().translate(-trail, fork_gap * 0.5 + 0.003, -(0.020 + fork_drop * 0.5)))
    geom.merge(plate.copy().translate(-trail, -(fork_gap * 0.5 + 0.003), -(0.020 + fork_drop * 0.5)))
    return geom


def _build_handle_mesh(*, width: float, height: float, rake: float, tube_radius: float) -> object:
    handle_path = [
        (0.0, -width * 0.5, 0.0),
        (-rake * 0.40, -width * 0.5, height * 0.22),
        (-rake * 0.78, -width * 0.5, height * 0.62),
        (-rake, -width * 0.5, height),
        (-rake, width * 0.5, height),
        (-rake * 0.78, width * 0.5, height * 0.62),
        (-rake * 0.40, width * 0.5, height * 0.22),
        (0.0, width * 0.5, 0.0),
    ]
    return tube_from_spline_points(
        handle_path,
        radius=tube_radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.16, 0.30, 0.57, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.73, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    gray_poly = model.material("gray_poly", rgba=(0.50, 0.52, 0.55, 1.0))

    deck_length = 0.68
    deck_width = 0.42
    deck_thickness = 0.025
    deck_top = 0.135
    deck_center_z = deck_top - deck_thickness * 0.5
    rear_wheel_radius = 0.080
    rear_wheel_width = 0.040
    front_wheel_radius = 0.040
    front_wheel_width = 0.024
    rear_axle_x = -0.250
    front_caster_x = 0.275
    caster_y = 0.145
    caster_trail = 0.060

    deck = model.part("deck")
    deck_shell = _save_mesh(
        "deck_shell",
        ExtrudeGeometry(
            rounded_rect_profile(deck_length, deck_width, 0.028, corner_segments=10),
            deck_thickness,
            center=True,
        ),
    )
    deck.visual(
        deck_shell,
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.54, 0.050, 0.018)),
        origin=Origin(xyz=(-0.010, 0.090, deck_center_z - 0.021)),
        material=dark_steel,
        name="left_stiffener",
    )
    deck.visual(
        Box((0.54, 0.050, 0.018)),
        origin=Origin(xyz=(-0.010, -0.090, deck_center_z - 0.021)),
        material=dark_steel,
        name="right_stiffener",
    )
    deck.visual(
        Box((0.58, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z - 0.028)),
        material=dark_steel,
        name="center_spine",
    )
    deck.visual(
        Cylinder(radius=0.013, length=0.390),
        origin=Origin(xyz=(rear_axle_x, 0.0, rear_wheel_radius), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        deck.visual(
            Box((0.050, 0.020, 0.090)),
            origin=Origin(
                xyz=(
                    rear_axle_x + 0.020,
                    side_sign * 0.205,
                    rear_wheel_radius + 0.005,
                )
            ),
            material=dark_steel,
            name=f"{side_name}_rear_wheel_bracket",
        )
        deck.visual(
            Cylinder(radius=0.010, length=0.038),
            origin=Origin(
                xyz=(
                    rear_axle_x,
                    side_sign * 0.215,
                    rear_wheel_radius,
                ),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"{side_name}_rear_axle_stub",
        )
        deck.visual(
            Box((0.070, 0.050, 0.010)),
            origin=Origin(
                xyz=(
                    front_caster_x,
                    side_sign * caster_y,
                    deck_center_z - 0.015,
                )
            ),
            material=dark_steel,
            name=f"{side_name}_caster_pad",
        )
        deck.visual(
            Box((0.020, 0.016, 0.040)),
            origin=Origin(
                xyz=(
                    -deck_length * 0.5 + 0.010,
                    side_sign * 0.130,
                    deck_top + 0.007,
                )
            ),
            material=dark_steel,
            name=f"{side_name}_handle_hinge_lug",
        )
    deck.visual(
        Box((0.040, 0.330, 0.018)),
        origin=Origin(xyz=(-deck_length * 0.5 + 0.020, 0.0, deck_center_z - 0.006)),
        material=dark_steel,
        name="rear_hinge_bar",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.72, 0.46, 0.18)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    handle = model.part("handle")
    handle.visual(
        _save_mesh(
            "handle_frame",
            _build_handle_mesh(width=0.310, height=0.690, rake=0.028, tube_radius=0.011),
        ),
        material=steel,
        name="handle_frame",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.250),
        origin=Origin(xyz=(-0.028, 0.0, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gray_poly,
        name="handle_grip",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.148, 0.005, ), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_pivot_pin",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, -0.148, 0.005), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_pivot_pin",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.09, 0.34, 0.72)),
        mass=2.7,
        origin=Origin(xyz=(-0.014, 0.0, 0.350)),
    )

    rear_wheel_mesh = _save_mesh(
        "rear_wheel_body",
        _wheel_mesh(
            overall_radius=rear_wheel_radius,
            tire_tube=0.018,
            width=rear_wheel_width,
            hub_radius=0.016,
            hub_width=0.022,
        ),
    )
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.visual(rear_wheel_mesh, material=rubber, name="wheel_body")
    rear_left_wheel.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=rear_wheel_radius, length=rear_wheel_width),
        mass=1.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.visual(rear_wheel_mesh, material=rubber, name="wheel_body")
    rear_right_wheel.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=rear_wheel_radius, length=rear_wheel_width),
        mass=1.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    caster_support_mesh = _save_mesh(
        "caster_support",
        _build_caster_support_mesh(trail=caster_trail, fork_gap=0.030, fork_drop=0.060),
    )
    front_left_caster = model.part("front_left_caster")
    front_left_caster.visual(caster_support_mesh, material=steel, name="caster_support")
    front_left_caster.inertial = Inertial.from_geometry(
        Box((0.070, 0.050, 0.120)),
        mass=0.8,
        origin=Origin(xyz=(-0.018, 0.0, -0.040)),
    )

    front_right_caster = model.part("front_right_caster")
    front_right_caster.visual(caster_support_mesh, material=steel, name="caster_support")
    front_right_caster.inertial = Inertial.from_geometry(
        Box((0.070, 0.050, 0.120)),
        mass=0.8,
        origin=Origin(xyz=(-0.018, 0.0, -0.040)),
    )

    front_wheel_mesh = _save_mesh(
        "front_wheel_body",
        _wheel_mesh(
            overall_radius=front_wheel_radius,
            tire_tube=0.010,
            width=front_wheel_width,
            hub_radius=0.009,
            hub_width=0.014,
        ),
    )
    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.visual(front_wheel_mesh, material=rubber, name="wheel_body")
    front_left_wheel.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=gray_poly,
        name="hub_cap",
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=front_wheel_radius, length=front_wheel_width),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.visual(front_wheel_mesh, material=rubber, name="wheel_body")
    front_right_wheel.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=gray_poly,
        name="hub_cap",
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=front_wheel_radius, length=front_wheel_width),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(-deck_length * 0.5 + 0.006, 0.0, deck_top + 0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.12,
            upper=1.28,
        ),
    )
    model.articulation(
        "deck_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_left_wheel,
        origin=Origin(xyz=(rear_axle_x, 0.245, rear_wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_right_wheel,
        origin=Origin(xyz=(rear_axle_x, -0.245, rear_wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_front_left_caster",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_left_caster,
        origin=Origin(xyz=(front_caster_x, caster_y, deck_center_z - deck_thickness * 0.5 - 0.0075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "deck_to_front_right_caster",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_right_caster,
        origin=Origin(xyz=(front_caster_x, -caster_y, deck_center_z - deck_thickness * 0.5 - 0.0075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "front_left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster,
        child=front_left_wheel,
        origin=Origin(xyz=(-caster_trail, 0.0, -0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "front_right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster,
        child=front_right_wheel,
        origin=Origin(xyz=(-caster_trail, 0.0, -0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
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
    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")

    handle_joint = object_model.get_articulation("deck_to_handle")
    rear_left_spin = object_model.get_articulation("deck_to_rear_left_wheel")
    rear_right_spin = object_model.get_articulation("deck_to_rear_right_wheel")
    front_left_swivel = object_model.get_articulation("deck_to_front_left_caster")
    front_right_swivel = object_model.get_articulation("deck_to_front_right_caster")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_spin = object_model.get_articulation("front_right_caster_to_wheel")

    ctx.allow_overlap(
        handle,
        deck,
        elem_a="left_pivot_pin",
        elem_b=LEFT_HANDLE_HINGE_LUG,
        reason="The folding handle rotates on steel pivot pins captured by simplified hinge lugs.",
    )
    ctx.allow_overlap(
        handle,
        deck,
        elem_a="right_pivot_pin",
        elem_b=RIGHT_HANDLE_HINGE_LUG,
        reason="The folding handle rotates on steel pivot pins captured by simplified hinge lugs.",
    )
    ctx.allow_overlap(
        rear_left_wheel,
        deck,
        elem_a="wheel_body",
        elem_b=LEFT_REAR_AXLE_STUB,
        reason="The rear wheel hub is simplified and intentionally shares volume with the axle stub.",
    )
    ctx.allow_overlap(
        object_model.get_part("rear_right_wheel"),
        deck,
        elem_a="wheel_body",
        elem_b=RIGHT_REAR_AXLE_STUB,
        reason="The rear wheel hub is simplified and intentionally shares volume with the axle stub.",
    )

    ctx.check(
        "handle hinge rotates about cart width axis",
        handle_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "rear wheels spin on fixed axle line",
        rear_left_spin.axis == (0.0, 1.0, 0.0) and rear_right_spin.axis == (0.0, 1.0, 0.0),
        details=f"left={rear_left_spin.axis}, right={rear_right_spin.axis}",
    )
    ctx.check(
        "caster supports swivel vertically",
        front_left_swivel.axis == (0.0, 0.0, 1.0) and front_right_swivel.axis == (0.0, 0.0, 1.0),
        details=f"left={front_left_swivel.axis}, right={front_right_swivel.axis}",
    )
    ctx.check(
        "caster wheels spin on fork axle",
        front_left_spin.axis == (0.0, 1.0, 0.0) and front_right_spin.axis == (0.0, 1.0, 0.0),
        details=f"left={front_left_spin.axis}, right={front_right_spin.axis}",
    )

    ctx.expect_contact(
        handle,
        deck,
        elem_a="left_pivot_pin",
        elem_b=LEFT_HANDLE_HINGE_LUG,
        contact_tol=0.0005,
        name="handle pivot pin seats in hinge lug",
    )
    ctx.expect_contact(
        rear_left_wheel,
        deck,
        elem_a="wheel_body",
        elem_b=LEFT_REAR_AXLE_STUB,
        contact_tol=0.0005,
        name="rear fixed wheel mounts on axle stub",
    )

    with ctx.pose({handle_joint: 0.0}):
        deployed_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        folded_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    handle_folds = (
        deployed_grip is not None
        and folded_grip is not None
        and folded_grip[1][2] < deployed_grip[1][2] - 0.25
        and folded_grip[1][0] > deployed_grip[1][0] + 0.20
    )
    ctx.check(
        "push handle folds down over the deck",
        handle_folds,
        details=f"deployed={deployed_grip}, folded={folded_grip}",
    )

    with ctx.pose({front_left_swivel: 0.0}):
        straight_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(front_left_wheel)
    caster_orbits = (
        straight_pos is not None
        and turned_pos is not None
        and abs(turned_pos[0] - straight_pos[0]) > 0.020
        and abs(turned_pos[1] - straight_pos[1]) > 0.020
    )
    ctx.check(
        "front caster wheel trails and orbits under swivel",
        caster_orbits,
        details=f"straight={straight_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
