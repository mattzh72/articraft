from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BARREL_LENGTH = 0.72
BARREL_RADIUS = 0.22
SHELL_THICKNESS = 0.014
CHAMBER_CENTER_Z = 0.69

FRAME_HALF_WIDTH = 0.28
FRAME_HALF_DEPTH = 0.18
FRAME_TUBE_RADIUS = 0.014
FRAME_TOP_Z = 0.432
FRAME_LOWER_Z = 0.12

WHEEL_RADIUS = 0.11
WHEEL_WIDTH = 0.048
WHEEL_AXLE_Y = -FRAME_HALF_DEPTH
WHEEL_AXLE_Z = WHEEL_RADIUS

SHELF_MOUNT_X = 0.45
SHELF_HEIGHT = 0.70


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _arc_profile(radius: float, start: float, end: float, samples: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.sin(start + (end - start) * i / (samples - 1)),
            radius * math.cos(start + (end - start) * i / (samples - 1)),
        )
        for i in range(samples)
    ]


def _barrel_half_mesh(*, upper: bool):
    inner_radius = BARREL_RADIUS - SHELL_THICKNESS
    start = 0.0 if upper else math.pi
    end = math.pi if upper else math.tau
    outer = _arc_profile(BARREL_RADIUS, start, end)
    inner = list(reversed(_arc_profile(inner_radius, start, end)))
    return ExtrudeGeometry(outer + inner, BARREL_LENGTH, center=True).rotate_y(math.pi / 2.0)


def _wheel_tire_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (WHEEL_RADIUS * 0.48, -half_width * 0.88),
        (WHEEL_RADIUS * 0.70, -half_width * 0.98),
        (WHEEL_RADIUS * 0.94, -half_width * 0.74),
        (WHEEL_RADIUS, -half_width * 0.24),
        (WHEEL_RADIUS, half_width * 0.24),
        (WHEEL_RADIUS * 0.94, half_width * 0.74),
        (WHEEL_RADIUS * 0.70, half_width * 0.98),
        (WHEEL_RADIUS * 0.48, half_width * 0.88),
        (WHEEL_RADIUS * 0.40, half_width * 0.26),
        (WHEEL_RADIUS * 0.38, 0.0),
        (WHEEL_RADIUS * 0.40, -half_width * 0.26),
        (WHEEL_RADIUS * 0.48, -half_width * 0.88),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def _wheel_side_sign(part_name: str) -> float:
    return -1.0 if "left" in part_name else 1.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_grill", assets=ASSETS)

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    wood = model.material("wood", rgba=(0.54, 0.38, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    lower_shell_mesh = _save_mesh("grill_lower_shell.obj", _barrel_half_mesh(upper=False))
    lid_shell_mesh = _save_mesh("grill_lid_shell.obj", _barrel_half_mesh(upper=True))
    wheel_tire_mesh = _save_mesh("grill_wheel_tire.obj", _wheel_tire_mesh())

    cart_frame = model.part("cart_frame")
    cart_frame.inertial = Inertial.from_geometry(
        Box((0.84, 0.40, 0.76)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )
    for x in (-FRAME_HALF_WIDTH, FRAME_HALF_WIDTH):
        for y in (-FRAME_HALF_DEPTH, FRAME_HALF_DEPTH):
            cart_frame.visual(
                Cylinder(radius=FRAME_TUBE_RADIUS, length=FRAME_TOP_Z),
                origin=Origin(xyz=(x, y, FRAME_TOP_Z * 0.5)),
                material=powder_black,
            )
    cart_frame.visual(
        Cylinder(radius=FRAME_TUBE_RADIUS, length=0.70),
        origin=Origin(xyz=(0.0, FRAME_HALF_DEPTH, FRAME_TOP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="front_top_rail",
    )
    cart_frame.visual(
        Cylinder(radius=FRAME_TUBE_RADIUS, length=0.70),
        origin=Origin(xyz=(0.0, -FRAME_HALF_DEPTH, FRAME_TOP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="rear_top_rail",
    )
    cart_frame.visual(
        Cylinder(radius=FRAME_TUBE_RADIUS, length=0.56),
        origin=Origin(xyz=(0.0, FRAME_HALF_DEPTH, FRAME_LOWER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="front_lower_rail",
    )
    cart_frame.visual(
        Cylinder(radius=FRAME_TUBE_RADIUS, length=0.36),
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, 0.0, FRAME_LOWER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=FRAME_TUBE_RADIUS, length=0.36),
        origin=Origin(xyz=(FRAME_HALF_WIDTH, 0.0, FRAME_LOWER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.62),
        origin=Origin(xyz=(0.0, WHEEL_AXLE_Y, WHEEL_AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )
    cart_frame.visual(
        Box((0.12, 0.12, 0.016)),
        origin=Origin(xyz=(-0.18, 0.0, FRAME_TOP_Z)),
        material=powder_black,
        name="left_cradle",
    )
    cart_frame.visual(
        Box((0.12, 0.12, 0.016)),
        origin=Origin(xyz=(0.18, 0.0, FRAME_TOP_Z)),
        material=powder_black,
        name="right_cradle",
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(-0.18, 0.12, FRAME_TOP_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(-0.18, -0.12, FRAME_TOP_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.18, 0.12, FRAME_TOP_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.18, -0.12, FRAME_TOP_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Box((0.06, 0.04, 0.04)),
        origin=Origin(xyz=(0.42, 0.10, SHELF_HEIGHT)),
        material=powder_black,
        name="shelf_front_pad",
    )
    cart_frame.visual(
        Box((0.06, 0.04, 0.04)),
        origin=Origin(xyz=(0.42, -0.10, SHELF_HEIGHT)),
        material=powder_black,
        name="shelf_rear_pad",
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=SHELF_HEIGHT - FRAME_TOP_Z),
        origin=Origin(xyz=(0.42, 0.10, 0.5 * (SHELF_HEIGHT + FRAME_TOP_Z))),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=SHELF_HEIGHT - FRAME_TOP_Z),
        origin=Origin(xyz=(0.42, -0.10, 0.5 * (SHELF_HEIGHT + FRAME_TOP_Z))),
        material=powder_black,
    )
    cart_frame.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.42, 0.0, SHELF_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    cart_frame.visual(
        Box((0.14, 0.08, 0.04)),
        origin=Origin(xyz=(0.385, 0.14, 0.44)),
        material=powder_black,
    )
    cart_frame.visual(
        Box((0.14, 0.08, 0.04)),
        origin=Origin(xyz=(0.385, -0.14, 0.44)),
        material=powder_black,
    )

    lower_chamber = model.part("lower_chamber")
    lower_chamber.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        mass=12.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    lower_chamber.visual(lid_shell_mesh, material=powder_black, name="lower_shell")
    lower_chamber.visual(
        Box((0.10, 0.10, 0.22)),
        origin=Origin(xyz=(-0.18, 0.0, -0.14)),
        material=dark_steel,
        name="left_mount_bracket",
    )
    lower_chamber.visual(
        Box((0.10, 0.10, 0.22)),
        origin=Origin(xyz=(0.18, 0.0, -0.14)),
        material=dark_steel,
        name="right_mount_bracket",
    )
    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((BARREL_LENGTH, 0.44, 0.26)),
        mass=7.0,
        origin=Origin(xyz=(0.0, BARREL_RADIUS, 0.10)),
    )
    lid.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, BARREL_RADIUS, 0.0)),
        material=powder_black,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(-0.16, BARREL_RADIUS * 1.86, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(0.16, BARREL_RADIUS * 1.86, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.34),
        origin=Origin(xyz=(0.0, BARREL_RADIUS * 2.02, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(-0.24, BARREL_RADIUS * 1.40, 0.214)),
        material=matte_black,
        name="chimney_base",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.24, BARREL_RADIUS * 1.40, 0.268)),
        material=matte_black,
        name="chimney_stack",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.24, BARREL_RADIUS * 1.40, 0.323)),
        material=matte_black,
    )

    side_shelf = model.part("side_shelf")
    side_shelf.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.08)),
        mass=3.0,
        origin=Origin(xyz=(0.18, 0.0, 0.02)),
    )
    side_shelf.visual(
        Box((0.28, 0.28, 0.02)),
        origin=Origin(xyz=(0.18, 0.0, 0.02)),
        material=wood,
        name="shelf_top",
    )
    side_shelf.visual(
        Box((0.12, 0.03, 0.02)),
        origin=Origin(xyz=(0.06, 0.10, 0.0)),
        material=powder_black,
        name="front_bracket",
    )
    side_shelf.visual(
        Box((0.12, 0.03, 0.02)),
        origin=Origin(xyz=(0.06, -0.10, 0.0)),
        material=powder_black,
        name="rear_bracket",
    )
    side_shelf.visual(
        Cylinder(radius=0.008, length=0.28),
        origin=Origin(xyz=(0.18, 0.0, 0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )

    for wheel_name, wheel_x in (("left_wheel", -0.31), ("right_wheel", 0.31)):
        sign = _wheel_side_sign(wheel_name)
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=2.2,
            origin=Origin(xyz=(sign * WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        wheel.visual(
            wheel_tire_mesh,
            origin=Origin(xyz=(sign * WHEEL_WIDTH * 0.5, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.072, length=0.016),
            origin=Origin(xyz=(sign * 0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.047, length=0.014),
            origin=Origin(xyz=(sign * 0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="hub_inner",
        )
        wheel.visual(
            Cylinder(radius=0.050, length=0.010),
            origin=Origin(xyz=(sign * 0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
        )
        wheel.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(sign * 0.020, 0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="valve_stem",
        )

    model.articulation(
        "frame_to_lower_chamber",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=lower_chamber,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_CENTER_Z)),
    )
    model.articulation(
        "lower_chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_chamber,
        child=lid,
        origin=Origin(xyz=(0.0, -BARREL_RADIUS, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "frame_to_side_shelf",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=side_shelf,
        origin=Origin(xyz=(SHELF_MOUNT_X, 0.0, SHELF_HEIGHT)),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child="left_wheel",
        origin=Origin(xyz=(-0.31, WHEEL_AXLE_Y, WHEEL_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child="right_wheel",
        origin=Origin(xyz=(0.31, WHEEL_AXLE_Y, WHEEL_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cart_frame = object_model.get_part("cart_frame")
    lower_chamber = object_model.get_part("lower_chamber")
    lid = object_model.get_part("lid")
    side_shelf = object_model.get_part("side_shelf")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("lower_chamber_to_lid")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    lower_shell = lower_chamber.get_visual("lower_shell")
    left_mount = lower_chamber.get_visual("left_mount_bracket")
    right_mount = lower_chamber.get_visual("right_mount_bracket")
    lid_shell = lid.get_visual("lid_shell")
    handle_bar = lid.get_visual("handle_bar")
    shelf_top = side_shelf.get_visual("shelf_top")
    front_bracket = side_shelf.get_visual("front_bracket")
    rear_bracket = side_shelf.get_visual("rear_bracket")
    axle = cart_frame.get_visual("wheel_axle")
    left_cradle = cart_frame.get_visual("left_cradle")
    right_cradle = cart_frame.get_visual("right_cradle")
    shelf_front_pad = cart_frame.get_visual("shelf_front_pad")
    shelf_rear_pad = cart_frame.get_visual("shelf_rear_pad")
    left_hub = left_wheel.get_visual("hub_inner")
    right_hub = right_wheel.get_visual("hub_inner")
    right_valve = right_wheel.get_visual("valve_stem")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        left_wheel,
        cart_frame,
        reason="Left wheel hub is intentionally sleeved onto the cart axle.",
        elem_a=left_hub,
        elem_b=axle,
    )
    ctx.allow_overlap(
        right_wheel,
        cart_frame,
        reason="Right wheel hub is intentionally sleeved onto the cart axle.",
        elem_a=right_hub,
        elem_b=axle,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(lower_chamber, cart_frame, elem_a=left_mount, elem_b=left_cradle)
    ctx.expect_contact(lower_chamber, cart_frame, elem_a=right_mount, elem_b=right_cradle)
    ctx.expect_overlap(lower_chamber, cart_frame, axes="xy", min_overlap=0.09, elem_a=left_mount, elem_b=left_cradle)
    ctx.expect_overlap(lower_chamber, cart_frame, axes="xy", min_overlap=0.09, elem_a=right_mount, elem_b=right_cradle)

    ctx.expect_contact(lid, lower_chamber, elem_a=lid_shell, elem_b=lower_shell)
    ctx.expect_overlap(lid, lower_chamber, axes="x", min_overlap=0.68, elem_a=lid_shell, elem_b=lower_shell)

    ctx.expect_contact(side_shelf, cart_frame, elem_a=front_bracket, elem_b=shelf_front_pad)
    ctx.expect_contact(side_shelf, cart_frame, elem_a=rear_bracket, elem_b=shelf_rear_pad)
    ctx.expect_gap(
        side_shelf,
        cart_frame,
        axis="x",
        positive_elem=front_bracket,
        negative_elem=shelf_front_pad,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        side_shelf,
        lower_chamber,
        axis="x",
        positive_elem=shelf_top,
        negative_elem=lower_shell,
        min_gap=0.03,
    )
    ctx.expect_origin_gap(side_shelf, lower_chamber, axis="x", min_gap=0.35)

    ctx.expect_contact(left_wheel, cart_frame, elem_a=left_hub, elem_b=axle)
    ctx.expect_contact(right_wheel, cart_frame, elem_a=right_hub, elem_b=axle)

    handle_rest = ctx.part_element_world_aabb(lid, elem=handle_bar)
    assert handle_rest is not None
    with ctx.pose({lid_hinge: math.radians(95.0)}):
        handle_open = ctx.part_element_world_aabb(lid, elem=handle_bar)
        assert handle_open is not None
        assert handle_open[1][2] > handle_rest[1][2] + 0.10
        assert handle_open[0][1] < handle_rest[0][1] - 0.08
        ctx.expect_gap(
            lid,
            lower_chamber,
            axis="z",
            positive_elem=handle_bar,
            negative_elem=lower_shell,
            min_gap=0.12,
        )

    valve_rest = ctx.part_element_world_aabb(right_wheel, elem=right_valve)
    assert valve_rest is not None
    with ctx.pose({right_wheel_spin: math.pi / 2.0}):
        valve_quarter_turn = ctx.part_element_world_aabb(right_wheel, elem=right_valve)
        assert valve_quarter_turn is not None
        assert valve_quarter_turn[1][2] > valve_rest[1][2] + 0.08
        ctx.expect_contact(right_wheel, cart_frame, elem_a=right_hub, elem_b=axle)
    with ctx.pose({left_wheel_spin: math.pi}):
        ctx.expect_contact(left_wheel, cart_frame, elem_a=left_hub, elem_b=axle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
