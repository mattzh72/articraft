from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bakery_deck_oven")

    body_paint = model.material("body_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    pedestal_paint = model.material("pedestal_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.34, 0.32, 0.29, 1.0))

    width = 1.42
    depth = 0.98
    front_y = depth / 2.0

    pedestal_w = 1.30
    pedestal_d = 0.86
    pedestal_h = 0.14

    shell_z0 = pedestal_h
    shell_h = 1.30
    shell_top = shell_z0 + shell_h

    side_t = 0.055
    rear_t = 0.040
    top_t = 0.045
    base_t = 0.080
    divider_t = 0.090
    frame_t = 0.030

    bottom_fascia = 0.160
    top_fascia = 0.150
    center_rail = 0.110
    opening_h = (shell_h - bottom_fascia - top_fascia - center_rail) / 2.0
    opening_w = 1.180
    side_post_w = (width - opening_w) / 2.0

    lower_open_z0 = shell_z0 + bottom_fascia
    lower_open_top = lower_open_z0 + opening_h
    upper_open_z0 = lower_open_top + center_rail

    clear_w = width - 2.0 * side_t
    cavity_depth = depth - rear_t - frame_t
    cavity_y = ((-depth / 2.0 + rear_t) + (front_y - frame_t)) / 2.0
    deck_t = 0.030

    body = model.part("body")
    body.visual(
        Box((pedestal_w, pedestal_d, pedestal_h)),
        origin=Origin(xyz=(0.0, -0.03, pedestal_h / 2.0)),
        material=pedestal_paint,
        name="pedestal",
    )
    body.visual(
        Box((width, depth, base_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_z0 + base_t / 2.0)),
        material=body_paint,
        name="base_pan",
    )
    body.visual(
        Box((side_t, depth, shell_h)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, shell_z0 + shell_h / 2.0)),
        material=body_paint,
        name="left_side",
    )
    body.visual(
        Box((side_t, depth, shell_h)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, shell_z0 + shell_h / 2.0)),
        material=body_paint,
        name="right_side",
    )
    body.visual(
        Box((clear_w, rear_t, shell_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + rear_t / 2.0, shell_z0 + shell_h / 2.0)),
        material=body_paint,
        name="rear_panel",
    )
    body.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_top - top_t / 2.0)),
        material=body_paint,
        name="top_cap",
    )
    body.visual(
        Box((clear_w, depth - rear_t, divider_t)),
        origin=Origin(
            xyz=(
                0.0,
                rear_t / 2.0,
                lower_open_top + divider_t / 2.0,
            )
        ),
        material=body_paint,
        name="deck_divider",
    )
    body.visual(
        Box((width, frame_t, bottom_fascia)),
        origin=Origin(xyz=(0.0, front_y - frame_t / 2.0, shell_z0 + bottom_fascia / 2.0)),
        material=body_paint,
        name="bottom_fascia",
    )
    body.visual(
        Box((width, frame_t, center_rail)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - frame_t / 2.0,
                lower_open_top + center_rail / 2.0,
            )
        ),
        material=body_paint,
        name="center_fascia",
    )
    body.visual(
        Box((width, frame_t, top_fascia)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - frame_t / 2.0,
                shell_top - top_fascia / 2.0,
            )
        ),
        material=body_paint,
        name="top_fascia",
    )
    body.visual(
        Box((side_post_w, frame_t, shell_h - top_fascia - bottom_fascia)),
        origin=Origin(
            xyz=(
                -width / 2.0 + side_post_w / 2.0,
                front_y - frame_t / 2.0,
                shell_z0 + bottom_fascia + (shell_h - top_fascia - bottom_fascia) / 2.0,
            )
        ),
        material=body_paint,
        name="left_front_post",
    )
    body.visual(
        Box((side_post_w, frame_t, shell_h - top_fascia - bottom_fascia)),
        origin=Origin(
            xyz=(
                width / 2.0 - side_post_w / 2.0,
                front_y - frame_t / 2.0,
                shell_z0 + bottom_fascia + (shell_h - top_fascia - bottom_fascia) / 2.0,
            )
        ),
        material=body_paint,
        name="right_front_post",
    )
    body.visual(
        Box((clear_w, cavity_depth, deck_t)),
        origin=Origin(xyz=(0.0, cavity_y, lower_open_z0 - deck_t / 2.0)),
        material=deck_stone,
        name="lower_deck",
    )
    body.visual(
        Box((clear_w, cavity_depth, deck_t)),
        origin=Origin(xyz=(0.0, cavity_y, upper_open_z0 - deck_t / 2.0)),
        material=deck_stone,
        name="upper_deck",
    )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, shell_top)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, shell_top / 2.0)),
    )

    door_w = 1.240
    door_h = 0.480
    door_t = 0.030
    rod_radius = 0.017
    rod_length = 0.940
    standoff_radius = 0.010
    standoff_length = 0.060
    handle_z = door_h * 0.63
    standoff_x = door_w * 0.34
    panel_w = door_w * 0.88
    panel_h = door_h * 0.72
    hinge_pin_radius = 0.012
    hinge_pin_length = 0.028
    hinge_pin_y = 0.065
    hinge_ear_w = 0.032
    hinge_ear_y = 0.032
    hinge_ear_h = 0.056
    hinge_ear_overlap = 0.002

    def make_door(part_name: str, hinge_name: str, hinge_z: float) -> None:
        door = model.part(part_name)
        door.visual(
            Box((door_w, door_t, door_h)),
            origin=Origin(xyz=(0.0, door_t / 2.0, door_h / 2.0)),
            material=stainless,
            name="door_leaf",
        )
        door.visual(
            Box((panel_w, 0.009, panel_h)),
            origin=Origin(xyz=(0.0, door_t + 0.0045, door_h * 0.52)),
            material=stainless,
            name="door_face_panel",
        )
        door.visual(
            Cylinder(radius=rod_radius, length=rod_length),
            origin=Origin(
                xyz=(0.0, door_t + standoff_length + rod_radius, handle_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=brushed_steel,
            name="handle_rod",
        )
        for side, x_pos in (("left", -standoff_x), ("right", standoff_x)):
            door.visual(
                Cylinder(radius=standoff_radius, length=standoff_length),
                origin=Origin(
                    xyz=(x_pos, door_t + standoff_length / 2.0, handle_z),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=brushed_steel,
                name=f"{side}_handle_post",
            )
        for side, sign in (("left", -1.0), ("right", 1.0)):
            ear_center_x = sign * (door_w / 2.0 + hinge_ear_w / 2.0 - hinge_ear_overlap)
            pin_center_x = sign * (door_w / 2.0 + hinge_pin_length / 2.0 + 0.002)
            door.visual(
                Box((hinge_ear_w, hinge_ear_y, hinge_ear_h)),
                origin=Origin(
                    xyz=(
                        ear_center_x,
                        (door_t + hinge_pin_y - hinge_pin_radius) / 2.0,
                        hinge_ear_h / 2.0,
                    )
                ),
                material=stainless,
                name=f"{side}_hinge_ear",
            )
            door.visual(
                Cylinder(radius=hinge_pin_radius, length=hinge_pin_length),
                origin=Origin(
                    xyz=(pin_center_x, hinge_pin_y, hinge_pin_radius),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=brushed_steel,
                name=f"{side}_hinge_pin",
            )
        door.inertial = Inertial.from_geometry(
            Box((door_w, door_t, door_h)),
            mass=26.0,
            origin=Origin(xyz=(0.0, door_t / 2.0, door_h / 2.0)),
        )
        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(0.0, front_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=260.0,
                velocity=1.2,
                lower=-1.45,
                upper=0.0,
            ),
        )

    make_door("lower_door", "lower_door_hinge", lower_open_z0 - 0.020)
    make_door("upper_door", "upper_door_hinge", upper_open_z0 - 0.020)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lower_door = object_model.get_part("lower_door")
    upper_door = object_model.get_part("upper_door")
    lower_hinge = object_model.get_articulation("lower_door_hinge")
    upper_hinge = object_model.get_articulation("upper_door_hinge")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))

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

    ctx.check("body part present", body.name == "body", f"Unexpected body name: {body.name}")
    ctx.check(
        "lower door part present",
        lower_door.name == "lower_door",
        f"Unexpected lower door name: {lower_door.name}",
    )
    ctx.check(
        "upper door part present",
        upper_door.name == "upper_door",
        f"Unexpected upper door name: {upper_door.name}",
    )
    ctx.check(
        "lower door hinge axis is widthwise",
        tuple(lower_hinge.axis) == (1.0, 0.0, 0.0),
        f"Axis was {lower_hinge.axis!r}",
    )
    ctx.check(
        "upper door hinge axis is widthwise",
        tuple(upper_hinge.axis) == (1.0, 0.0, 0.0),
        f"Axis was {upper_hinge.axis!r}",
    )
    ctx.check(
        "lower door hinge opens downward only",
        lower_hinge.motion_limits is not None
        and lower_hinge.motion_limits.lower is not None
        and lower_hinge.motion_limits.upper is not None
        and lower_hinge.motion_limits.lower <= -1.3
        and abs(lower_hinge.motion_limits.upper) <= 1e-9,
        f"Limits were {lower_hinge.motion_limits!r}",
    )
    ctx.check(
        "upper door hinge opens downward only",
        upper_hinge.motion_limits is not None
        and upper_hinge.motion_limits.lower is not None
        and upper_hinge.motion_limits.upper is not None
        and upper_hinge.motion_limits.lower <= -1.3
        and abs(upper_hinge.motion_limits.upper) <= 1e-9,
        f"Limits were {upper_hinge.motion_limits!r}",
    )

    with ctx.pose({lower_hinge: 0.0, upper_hinge: 0.0}):
        ctx.expect_gap(
            lower_door,
            body,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="door_leaf",
            name="lower door closes flush to the oven face",
        )
        ctx.expect_gap(
            upper_door,
            body,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="door_leaf",
            name="upper door closes flush to the oven face",
        )
        ctx.expect_within(
            lower_door,
            body,
            axes="x",
            margin=0.02,
            inner_elem="door_leaf",
            name="lower door stays within body width",
        )
        ctx.expect_within(
            upper_door,
            body,
            axes="x",
            margin=0.02,
            inner_elem="door_leaf",
            name="upper door stays within body width",
        )
        ctx.expect_origin_gap(
            upper_door,
            lower_door,
            axis="z",
            min_gap=0.45,
            max_gap=0.65,
            name="two deck doors are vertically stacked",
        )
        lower_closed = aabb_center(ctx.part_element_world_aabb(lower_door, elem="door_leaf"))
        upper_closed = aabb_center(ctx.part_element_world_aabb(upper_door, elem="door_leaf"))

    with ctx.pose({lower_hinge: -1.35, upper_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lower door clears the cabinet when open")
        lower_open = aabb_center(ctx.part_element_world_aabb(lower_door, elem="door_leaf"))
    ctx.check(
        "lower door swings outward and down",
        lower_closed is not None
        and lower_open is not None
        and lower_open[1] > lower_closed[1] + 0.18
        and lower_open[2] < lower_closed[2] - 0.12,
        f"Closed center={lower_closed}, open center={lower_open}",
    )

    with ctx.pose({lower_hinge: 0.0, upper_hinge: -1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="upper door clears the cabinet when open")
        upper_open = aabb_center(ctx.part_element_world_aabb(upper_door, elem="door_leaf"))
    ctx.check(
        "upper door swings outward and down",
        upper_closed is not None
        and upper_open is not None
        and upper_open[1] > upper_closed[1] + 0.18
        and upper_open[2] < upper_closed[2] - 0.12,
        f"Closed center={upper_closed}, open center={upper_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
