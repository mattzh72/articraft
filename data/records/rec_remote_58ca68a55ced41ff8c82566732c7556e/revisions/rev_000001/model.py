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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _linspace(start: float, stop: float, steps: int) -> list[float]:
    if steps <= 1:
        return [start]
    return [start + (stop - start) * i / (steps - 1) for i in range(steps)]


def _guard_section_loop(
    x_pos: float,
    *,
    center_y: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    angle_start: float,
    angle_end: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    outer = [
        (
            x_pos,
            center_y + outer_radius * math.sin(theta),
            center_z + outer_radius * math.cos(theta),
        )
        for theta in _linspace(angle_start, angle_end, samples)
    ]
    inner = [
        (
            x_pos,
            center_y + inner_radius * math.sin(theta),
            center_z + inner_radius * math.cos(theta),
        )
        for theta in _linspace(angle_end, angle_start, samples)
    ]
    return outer + inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_laser_clicker")

    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.08, 0.09, 1.0))
    cap_graphite = model.material("cap_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    button_red = model.material("button_red", rgba=(0.73, 0.12, 0.10, 1.0))
    lens_red = model.material("lens_red", rgba=(0.70, 0.08, 0.08, 1.0))
    guard_smoke = model.material("guard_smoke", rgba=(0.32, 0.35, 0.39, 0.88))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))

    body = model.part("body")

    body_profile = [
        (0.0, -0.056),
        (0.0068, -0.056),
        (0.0072, -0.050),
        (0.0075, -0.012),
        (0.0074, 0.028),
        (0.0070, 0.053),
        (0.0060, 0.060),
        (0.0032, 0.064),
        (0.0, 0.064),
    ]
    body_shell = mesh_from_geometry(
        LatheGeometry(body_profile, segments=56).rotate_y(math.pi / 2.0),
        "clicker_body_shell",
    )
    body.visual(body_shell, material=body_black, name="body_shell")
    body.visual(
        Box((0.050, 0.010, 0.0016)),
        origin=Origin(xyz=(0.014, 0.0, 0.0080)),
        material=grip_black,
        name="top_saddle",
    )
    body.visual(
        Cylinder(radius=0.0077, length=0.004),
        origin=Origin(xyz=(-0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_graphite,
        name="rear_collar",
    )
    body.visual(
        Cylinder(radius=0.0036, length=0.005),
        origin=Origin(xyz=(0.0615, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="emitter_bezel",
    )
    body.visual(
        Cylinder(radius=0.0017, length=0.0015),
        origin=Origin(xyz=(0.0642, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_red,
        name="laser_lens",
    )

    guard_center_y = 0.0015
    guard_center_z = 0.0025
    guard_inner_radius = 0.0102
    guard_outer_radius = 0.0118
    guard_angle_start = -1.02
    guard_angle_end = 0.80
    guard_hinge_radius = 0.0090
    guard_joint_x = 0.013
    guard_joint_y = guard_center_y + guard_hinge_radius * math.sin(guard_angle_start)
    guard_joint_z = guard_center_z + guard_hinge_radius * math.cos(guard_angle_start)

    for side, x_pos in (("left", guard_joint_x - 0.016), ("right", guard_joint_x + 0.016)):
        body.visual(
            Cylinder(radius=0.0017, length=0.014),
            origin=Origin(xyz=(x_pos, guard_joint_y, guard_joint_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cap_graphite,
            name=f"hinge_barrel_{side}",
        )
        body.visual(
            Box((0.011, 0.0028, 0.0040)),
            origin=Origin(xyz=(x_pos, guard_joint_y + 0.0009, guard_joint_z - 0.0017)),
            material=body_black,
            name=f"hinge_rib_{side}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.130, 0.020, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    pointer_button = model.part("pointer_button")
    pointer_button.visual(
        Box((0.014, 0.007, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=button_red,
        name="button_base",
    )
    pointer_button.visual(
        Cylinder(radius=0.0016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_red,
        name="button_crown",
    )
    pointer_button.visual(
        Cylinder(radius=0.0016, length=0.0032),
        origin=Origin(xyz=(-0.005, 0.0, 0.0014)),
        material=button_red,
        name="button_tip_left",
    )
    pointer_button.visual(
        Cylinder(radius=0.0016, length=0.0032),
        origin=Origin(xyz=(0.005, 0.0, 0.0014)),
        material=button_red,
        name="button_tip_right",
    )
    pointer_button.inertial = Inertial.from_geometry(
        Box((0.014, 0.007, 0.0035)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.00175)),
    )

    guard = model.part("thumb_guard")
    guard_loop_front = _guard_section_loop(
        -0.017,
        center_y=guard_center_y,
        center_z=guard_center_z,
        inner_radius=guard_inner_radius,
        outer_radius=guard_outer_radius,
        angle_start=guard_angle_start,
        angle_end=guard_angle_end,
    )
    guard_loop_mid = _guard_section_loop(
        0.0,
        center_y=guard_center_y,
        center_z=guard_center_z,
        inner_radius=guard_inner_radius,
        outer_radius=guard_outer_radius,
        angle_start=guard_angle_start,
        angle_end=guard_angle_end,
    )
    guard_loop_rear = _guard_section_loop(
        0.017,
        center_y=guard_center_y,
        center_z=guard_center_z,
        inner_radius=guard_inner_radius,
        outer_radius=guard_outer_radius,
        angle_start=guard_angle_start,
        angle_end=guard_angle_end,
    )
    guard_mesh = mesh_from_geometry(
        section_loft([guard_loop_front, guard_loop_mid, guard_loop_rear]),
        "thumb_guard_shell",
    )
    guard.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, -guard_joint_y, -guard_joint_z)),
        material=guard_smoke,
        name="guard_shell",
    )
    guard.visual(
        Cylinder(radius=0.0017, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_graphite,
        name="guard_barrel",
    )
    for name, x_pos in (("strap_front", -0.0055), ("strap_rear", 0.0055)):
        guard.visual(
            Box((0.007, 0.0016, 0.0032)),
            origin=Origin(xyz=(x_pos, -0.0011, 0.0009)),
            material=cap_graphite,
            name=name,
        )
    guard.inertial = Inertial.from_geometry(
        Box((0.040, 0.020, 0.014)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.003, 0.004)),
    )

    end_cap = model.part("battery_end_cap")
    cap_profile = [
        (0.0, -0.014),
        (0.0068, -0.014),
        (0.0074, -0.010),
        (0.0074, -0.003),
        (0.0072, 0.0),
        (0.0, 0.0),
    ]
    cap_mesh = mesh_from_geometry(
        LatheGeometry(cap_profile, segments=48).rotate_y(math.pi / 2.0),
        "battery_end_cap_shell",
    )
    end_cap.visual(cap_mesh, material=cap_graphite, name="cap_shell")
    end_cap.visual(
        Box((0.0045, 0.0025, 0.0016)),
        origin=Origin(xyz=(-0.0065, 0.0, 0.0076)),
        material=steel,
        name="cap_rib",
    )
    end_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.014),
        mass=0.015,
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_button",
        ArticulationType.FIXED,
        parent=body,
        child=pointer_button,
        origin=Origin(xyz=(0.014, 0.0, 0.0088)),
    )
    model.articulation(
        "body_to_thumb_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(guard_joint_x, guard_joint_y, guard_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.5,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "body_to_battery_end_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=end_cap,
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=-1.5 * math.pi,
            upper=1.5 * math.pi,
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
    body = object_model.get_part("body")
    pointer_button = object_model.get_part("pointer_button")
    guard = object_model.get_part("thumb_guard")
    end_cap = object_model.get_part("battery_end_cap")
    guard_hinge = object_model.get_articulation("body_to_thumb_guard")
    cap_joint = object_model.get_articulation("body_to_battery_end_cap")

    def elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({guard_hinge: 0.0, cap_joint: 0.0}):
        ctx.expect_contact(
            pointer_button,
            body,
            elem_a="button_base",
            elem_b="top_saddle",
            contact_tol=0.0002,
            name="pointer button is seated on the top saddle",
        )
        ctx.expect_contact(
            guard,
            body,
            contact_tol=0.0002,
            name="thumb guard hinge stays mounted to the barrel",
        )
        ctx.expect_overlap(
            guard,
            pointer_button,
            axes="xy",
            min_overlap=0.006,
            name="closed thumb guard covers the pointer button footprint",
        )
        ctx.expect_contact(
            guard,
            pointer_button,
            elem_a="guard_shell",
            elem_b="button_crown",
            contact_tol=0.0012,
            name="closed thumb guard sits closely over the pointer button",
        )
        ctx.expect_contact(
            end_cap,
            body,
            contact_tol=0.0002,
            name="battery end cap seats against the rear of the body",
        )
        ctx.expect_overlap(
            end_cap,
            body,
            axes="yz",
            min_overlap=0.010,
            name="battery end cap stays coaxial with the body",
        )
        guard_closed = elem_center(guard, "guard_shell")
        rib_closed = elem_center(end_cap, "cap_rib")

    with ctx.pose({guard_hinge: math.radians(105.0)}):
        guard_open = elem_center(guard, "guard_shell")

    ctx.check(
        "thumb guard lifts upward when opened",
        guard_closed is not None
        and guard_open is not None
        and guard_open[1] < guard_closed[1] - 0.008
        and guard_open[2] > guard_closed[2] + 0.001,
        details=f"closed={guard_closed}, open={guard_open}",
    )

    with ctx.pose({cap_joint: math.pi / 2.0}):
        ctx.expect_contact(
            end_cap,
            body,
            contact_tol=0.0002,
            name="battery end cap remains seated while rotated",
        )
        rib_rotated = elem_center(end_cap, "cap_rib")

    ctx.check(
        "battery cap rib moves around the body axis",
        rib_closed is not None
        and rib_rotated is not None
        and abs(rib_rotated[1] - rib_closed[1]) > 0.004
        and abs(rib_rotated[2] - rib_closed[2]) > 0.004,
        details=f"closed={rib_closed}, rotated={rib_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
