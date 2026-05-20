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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(x0: float, x1: float, y0: float, y1: float) -> list[tuple[float, float]]:
    return [
        (x0, y0),
        (x1, y0),
        (x1, y1),
        (x0, y1),
    ]


def _door_skin_mesh(
    *,
    door_x0: float,
    door_width: float,
    door_height: float,
    thickness: float,
) :
    outer = _rect_loop(door_x0, door_x0 + door_width, -door_height * 0.5, door_height * 0.5)

    slot_margin_left = 0.090
    slot_margin_right = 0.085
    slot_width = door_width - slot_margin_left - slot_margin_right
    slot_height = 0.012
    slot_pitch = 0.030
    first_center = door_height * 0.22
    holes = []
    for index in range(6):
        center_y = first_center + index * slot_pitch
        holes.append(
            _rect_loop(
                door_x0 + slot_margin_left,
                door_x0 + slot_margin_left + slot_width,
                center_y - slot_height * 0.5,
                center_y + slot_height * 0.5,
            )
        )

    panel = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=holes,
        height=thickness,
        center=True,
    )
    panel.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(panel, "locker_door_skin")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vented_locker_cabinet")

    body_color = model.material("body_grey", rgba=(0.56, 0.60, 0.62, 1.0))
    door_color = model.material("door_grey", rgba=(0.66, 0.69, 0.71, 1.0))
    hinge_color = model.material("hinge_metal", rgba=(0.45, 0.47, 0.49, 1.0))
    foot_color = model.material("foot_black", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_color = model.material("handle_dark", rgba=(0.17, 0.18, 0.19, 1.0))

    locker_w = 0.42
    locker_d = 0.48
    shell_h = 1.72
    foot_h = 0.05
    sheet_t = 0.018
    door_t = 0.022
    door_skin_t = 0.0026
    barrel_r = 0.012
    hinge_axis_x = -locker_w * 0.5 - 0.008
    hinge_axis_y = locker_d * 0.5 + door_t * 0.5 + 0.001
    door_x0 = 0.014
    door_w = 0.414
    door_h = shell_h - 0.038
    stile_w = 0.020
    hinge_cluster_h = 0.108
    hinge_segment_h = hinge_cluster_h / 3.0
    hinge_zs = {
        "upper": 0.58,
        "middle": 0.00,
        "lower": -0.58,
    }

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((sheet_t, locker_d, shell_h)),
        origin=Origin(xyz=(-locker_w * 0.5 + sheet_t * 0.5, 0.0, 0.0)),
        material=body_color,
        name="left_side",
    )
    cabinet.visual(
        Box((sheet_t, locker_d, shell_h)),
        origin=Origin(xyz=(locker_w * 0.5 - sheet_t * 0.5, 0.0, 0.0)),
        material=body_color,
        name="right_side",
    )
    cabinet.visual(
        Box((locker_w - 2.0 * sheet_t, sheet_t, shell_h)),
        origin=Origin(xyz=(0.0, -locker_d * 0.5 + sheet_t * 0.5, 0.0)),
        material=body_color,
        name="back_panel",
    )
    cabinet.visual(
        Box((locker_w - 2.0 * sheet_t, locker_d - sheet_t, sheet_t)),
        origin=Origin(xyz=(0.0, -sheet_t * 0.5, shell_h * 0.5 - sheet_t * 0.5)),
        material=body_color,
        name="roof_panel",
    )
    cabinet.visual(
        Box((locker_w - 2.0 * sheet_t, locker_d - sheet_t, sheet_t)),
        origin=Origin(xyz=(0.0, -sheet_t * 0.5, -shell_h * 0.5 + sheet_t * 0.5)),
        material=body_color,
        name="floor_panel",
    )
    cabinet.visual(
        Box((sheet_t, sheet_t, shell_h - 2.0 * sheet_t)),
        origin=Origin(
            xyz=(-locker_w * 0.5 + sheet_t * 0.5, locker_d * 0.5 - sheet_t * 0.5, 0.0)
        ),
        material=body_color,
        name="left_jamb",
    )
    cabinet.visual(
        Box((sheet_t, sheet_t, shell_h - 2.0 * sheet_t)),
        origin=Origin(
            xyz=(locker_w * 0.5 - sheet_t * 0.5, locker_d * 0.5 - sheet_t * 0.5, 0.0)
        ),
        material=body_color,
        name="right_jamb",
    )
    cabinet.visual(
        Box((locker_w - 2.0 * sheet_t, sheet_t, sheet_t)),
        origin=Origin(xyz=(0.0, locker_d * 0.5 - sheet_t * 0.5, shell_h * 0.5 - sheet_t * 0.5)),
        material=body_color,
        name="front_header",
    )
    cabinet.visual(
        Box((locker_w - 2.0 * sheet_t, sheet_t, sheet_t)),
        origin=Origin(xyz=(0.0, locker_d * 0.5 - sheet_t * 0.5, -shell_h * 0.5 + sheet_t * 0.5)),
        material=body_color,
        name="front_sill",
    )

    foot_size = 0.05
    foot_x = locker_w * 0.5 - foot_size * 0.7
    foot_y = locker_d * 0.5 - foot_size * 0.7
    foot_z = -shell_h * 0.5 - foot_h * 0.5
    for name, sx, sy in (
        ("front_left_foot", -foot_x, foot_y),
        ("front_right_foot", foot_x, foot_y),
        ("rear_left_foot", -foot_x, -foot_y),
        ("rear_right_foot", foot_x, -foot_y),
    ):
        cabinet.visual(
            Box((foot_size, foot_size, foot_h)),
            origin=Origin(xyz=(sx, sy, foot_z)),
            material=foot_color,
            name=name,
        )

    for hinge_name, hinge_z in hinge_zs.items():
        cabinet.visual(
            Cylinder(radius=barrel_r, length=hinge_segment_h),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_z - hinge_segment_h)),
            material=hinge_color,
            name=f"{hinge_name}_body_lower_knuckle",
        )
        cabinet.visual(
            Cylinder(radius=barrel_r, length=hinge_segment_h),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_z + hinge_segment_h)),
            material=hinge_color,
            name=f"{hinge_name}_body_upper_knuckle",
        )
        cabinet.visual(
            Box((0.012, 0.016, hinge_segment_h)),
            origin=Origin(xyz=(hinge_axis_x + 0.006, hinge_axis_y - 0.004, hinge_z - hinge_segment_h)),
            material=hinge_color,
            name=f"{hinge_name}_body_lower_leaf",
        )
        cabinet.visual(
            Box((0.012, 0.016, hinge_segment_h)),
            origin=Origin(xyz=(hinge_axis_x + 0.006, hinge_axis_y - 0.004, hinge_z + hinge_segment_h)),
            material=hinge_color,
            name=f"{hinge_name}_body_upper_leaf",
        )

    door = model.part("door")
    door.visual(
        _door_skin_mesh(
            door_x0=door_x0,
            door_width=door_w,
            door_height=door_h,
            thickness=door_skin_t,
        ),
        origin=Origin(xyz=(0.0, door_t * 0.5 - door_skin_t * 0.5, 0.0)),
        material=door_color,
        name="door_skin",
    )
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(xyz=(door_x0 + stile_w * 0.5, 0.0, 0.0)),
        material=door_color,
        name="left_return",
    )
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(xyz=(door_x0 + door_w - stile_w * 0.5, 0.0, 0.0)),
        material=door_color,
        name="right_return",
    )
    door.visual(
        Box((door_w, door_t, stile_w)),
        origin=Origin(xyz=(door_x0 + door_w * 0.5, 0.0, door_h * 0.5 - stile_w * 0.5)),
        material=door_color,
        name="top_return",
    )
    door.visual(
        Box((door_w, door_t, stile_w)),
        origin=Origin(xyz=(door_x0 + door_w * 0.5, 0.0, -door_h * 0.5 + stile_w * 0.5)),
        material=door_color,
        name="bottom_return",
    )

    handle_x = door_x0 + door_w - 0.065
    door.visual(
        Box((0.056, 0.002, 0.160)),
        origin=Origin(xyz=(handle_x, door_t * 0.5 + 0.001, 0.0)),
        material=hinge_color,
        name="latch_plate",
    )

    for hinge_name, hinge_z in hinge_zs.items():
        door.visual(
            Cylinder(radius=barrel_r, length=hinge_segment_h),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=hinge_color,
            name=f"{hinge_name}_door_knuckle",
        )
        door.visual(
            Box((0.028, 0.010, hinge_segment_h)),
            origin=Origin(xyz=(0.014, 0.006, hinge_z)),
            material=hinge_color,
            name=f"{hinge_name}_door_leaf",
        )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_color,
        name="hub",
    )
    handle.visual(
        Box((0.018, 0.014, 0.136)),
        origin=Origin(xyz=(0.030, 0.004, 0.0)),
        material=handle_color,
        name="grip",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(
            xyz=(0.042, 0.004, -0.054),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_color,
        name="end_knob",
    )

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    handle_axle = model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(handle_x, door_t * 0.5 + 0.011, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-1.15,
            upper=1.15,
        ),
    )

    model.meta["primary_articulations"] = (door_hinge.name, handle_axle.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_axle = object_model.get_articulation("door_to_latch_handle")

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
        "door hinge axis is vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {door_hinge.axis!r}",
    )
    ctx.check(
        "handle axle faces out of the door",
        tuple(handle_axle.axis) == (0.0, 1.0, 0.0),
        details=f"Expected handle axis normal to door, got {handle_axle.axis!r}",
    )

    door_limits = door_hinge.motion_limits
    handle_limits = handle_axle.motion_limits
    ctx.check(
        "door has realistic swing range",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper >= math.radians(100.0),
        details=f"Door limits were {door_limits!r}",
    )
    ctx.check(
        "handle has realistic twist range",
        handle_limits is not None
        and handle_limits.lower is not None
        and handle_limits.upper is not None
        and handle_limits.lower <= -1.0
        and handle_limits.upper >= 1.0,
        details=f"Handle limits were {handle_limits!r}",
    )

    with ctx.pose({door_hinge: 0.0, handle_axle: 0.0}):
        ctx.expect_contact(
            cabinet,
            door,
            elem_a="upper_body_lower_knuckle",
            elem_b="upper_door_knuckle",
            name="upper hinge knuckle stays clipped closed",
        )
        ctx.expect_contact(
            cabinet,
            door,
            elem_a="lower_body_upper_knuckle",
            elem_b="lower_door_knuckle",
            name="lower hinge knuckle stays clipped closed",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="hub",
            elem_b="latch_plate",
            name="handle hub mounts against latch plate",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="xz",
            min_overlap=0.30,
            name="closed door covers cabinet opening footprint",
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.001,
            max_gap=0.0045,
            positive_elem="right_return",
            negative_elem="right_jamb",
            name="latch stile stays close to cabinet face",
        )
        ctx.expect_origin_gap(
            handle,
            door,
            axis="x",
            min_gap=0.30,
            max_gap=0.39,
            name="handle sits near latch side",
        )
        ctx.expect_origin_gap(
            handle,
            door,
            axis="z",
            min_gap=-0.02,
            max_gap=0.02,
            name="handle sits at mid height",
        )

    with ctx.pose({door_hinge: math.radians(75.0)}):
        ctx.expect_contact(
            cabinet,
            door,
            elem_a="upper_body_lower_knuckle",
            elem_b="upper_door_knuckle",
            name="upper hinge knuckle stays clipped open",
        )
        ctx.expect_contact(
            cabinet,
            door,
            elem_a="lower_body_upper_knuckle",
            elem_b="lower_door_knuckle",
            name="lower hinge knuckle stays clipped open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
