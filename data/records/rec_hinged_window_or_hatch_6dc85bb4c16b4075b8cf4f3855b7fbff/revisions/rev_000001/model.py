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
    model = ArticulatedObject(name="casement_window")

    painted_aluminum = model.material("painted_aluminum", rgba=(0.93, 0.94, 0.95, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.12, 0.12, 0.12, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.23, 0.25, 0.28, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.70, 0.84, 0.92, 0.35))

    frame_width = 1.00
    frame_height = 1.40
    frame_depth = 0.12
    frame_member = 0.07

    opening_width = frame_width - 2.0 * frame_member
    opening_height = frame_height - 2.0 * frame_member

    sash_width = 0.852
    sash_height = 1.252
    sash_depth = 0.045
    sash_member = 0.06
    perimeter_gap = 0.004

    hinge_axis_y = 0.032
    sash_center_x = perimeter_gap + 0.5 * sash_width
    sash_center_y = -0.5 * sash_depth

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=34.0,
    )
    frame.visual(
        Box((frame_member, frame_depth, frame_height)),
        origin=Origin(xyz=(-0.5 * frame_width + 0.5 * frame_member, 0.0, 0.0)),
        material=painted_aluminum,
        name="frame_left_jamb",
    )
    frame.visual(
        Box((frame_member, frame_depth, frame_height)),
        origin=Origin(xyz=(0.5 * frame_width - 0.5 * frame_member, 0.0, 0.0)),
        material=painted_aluminum,
        name="frame_right_jamb",
    )
    frame.visual(
        Box((frame_width, frame_depth, frame_member)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * frame_height - 0.5 * frame_member)),
        material=painted_aluminum,
        name="frame_head",
    )
    frame.visual(
        Box((frame_width, frame_depth, frame_member)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * frame_height + 0.5 * frame_member)),
        material=painted_aluminum,
        name="frame_sill",
    )

    stop_depth = 0.012
    stop_projection = 0.018
    stop_center_y = -0.033
    frame.visual(
        Box((stop_projection, stop_depth, opening_height)),
        origin=Origin(
            xyz=(
                -0.5 * opening_width + 0.5 * stop_projection,
                stop_center_y,
                0.0,
            )
        ),
        material=painted_aluminum,
        name="frame_hinge_stop",
    )
    frame.visual(
        Box((stop_projection, stop_depth, opening_height)),
        origin=Origin(
            xyz=(
                0.5 * opening_width - 0.5 * stop_projection,
                stop_center_y,
                0.0,
            )
        ),
        material=painted_aluminum,
        name="frame_latch_stop",
    )
    frame.visual(
        Box((opening_width - 2.0 * stop_projection, stop_depth, stop_projection)),
        origin=Origin(
            xyz=(
                0.0,
                stop_center_y,
                0.5 * opening_height - 0.5 * stop_projection,
            )
        ),
        material=painted_aluminum,
        name="frame_head_stop",
    )
    frame.visual(
        Box((opening_width - 2.0 * stop_projection, stop_depth, stop_projection)),
        origin=Origin(
            xyz=(
                0.0,
                stop_center_y,
                -0.5 * opening_height + 0.5 * stop_projection,
            )
        ),
        material=painted_aluminum,
        name="frame_sill_stop",
    )

    hinge_plate_size = (0.010, 0.038, 0.18)
    hinge_plate_x = -0.5 * opening_width - 0.005
    hinge_plate_y = hinge_axis_y - 0.019
    upper_hinge_z = 0.40
    lower_hinge_z = -0.40
    frame.visual(
        Box(hinge_plate_size),
        origin=Origin(xyz=(hinge_plate_x, hinge_plate_y, upper_hinge_z)),
        material=hinge_steel,
        name="upper_frame_hinge_leaf",
    )
    frame.visual(
        Box(hinge_plate_size),
        origin=Origin(xyz=(hinge_plate_x, hinge_plate_y, lower_hinge_z)),
        material=hinge_steel,
        name="lower_frame_hinge_leaf",
    )
    frame.visual(
        Cylinder(radius=0.0035, length=0.16),
        origin=Origin(xyz=(-0.5 * opening_width, hinge_axis_y + 0.0045, upper_hinge_z), rpy=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="upper_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.0035, length=0.16),
        origin=Origin(xyz=(-0.5 * opening_width, hinge_axis_y + 0.0045, lower_hinge_z), rpy=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="lower_hinge_barrel",
    )

    sash = model.part("sash")
    sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=21.0,
        origin=Origin(xyz=(sash_center_x, sash_center_y, 0.0)),
    )
    sash.visual(
        Box((sash_member, sash_depth, sash_height)),
        origin=Origin(xyz=(perimeter_gap + 0.5 * sash_member, sash_center_y, 0.0)),
        material=painted_aluminum,
        name="sash_left_stile",
    )
    sash.visual(
        Box((sash_member, sash_depth, sash_height)),
        origin=Origin(xyz=(perimeter_gap + sash_width - 0.5 * sash_member, sash_center_y, 0.0)),
        material=painted_aluminum,
        name="sash_right_stile",
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_member)),
        origin=Origin(xyz=(sash_center_x, sash_center_y, 0.5 * sash_height - 0.5 * sash_member)),
        material=painted_aluminum,
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_member)),
        origin=Origin(xyz=(sash_center_x, sash_center_y, -0.5 * sash_height + 0.5 * sash_member)),
        material=painted_aluminum,
        name="sash_bottom_rail",
    )

    visible_glass_width = sash_width - 2.0 * sash_member
    visible_glass_height = sash_height - 2.0 * sash_member
    sash.visual(
        Box((visible_glass_width + 0.040, 0.018, visible_glass_height + 0.040)),
        origin=Origin(xyz=(sash_center_x, -0.023, 0.0)),
        material=glass_tint,
        name="insulated_glass",
    )
    sash.visual(
        Box((visible_glass_width + 0.020, 0.008, visible_glass_height + 0.020)),
        origin=Origin(xyz=(sash_center_x, -0.003, 0.0)),
        material=gasket_black,
        name="glazing_gasket",
    )

    sash_hinge_leaf_size = (0.012, 0.034, 0.16)
    sash.visual(
        Box(sash_hinge_leaf_size),
        origin=Origin(xyz=(0.006, -0.014, upper_hinge_z)),
        material=hinge_steel,
        name="upper_sash_hinge_leaf",
    )
    sash.visual(
        Box(sash_hinge_leaf_size),
        origin=Origin(xyz=(0.006, -0.014, lower_hinge_z)),
        material=hinge_steel,
        name="lower_sash_hinge_leaf",
    )

    handle_pivot_x = perimeter_gap + sash_width - 0.048
    handle_pivot_z = 0.0
    sash.visual(
        Box((0.034, 0.006, 0.190)),
        origin=Origin(xyz=(handle_pivot_x, 0.003, handle_pivot_z)),
        material=handle_metal,
        name="handle_plate",
    )
    sash.visual(
        Cylinder(radius=0.0125, length=0.006),
        origin=Origin(xyz=(handle_pivot_x, 0.006, handle_pivot_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="handle_base_boss",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.14, 0.028, 0.14)),
        mass=0.5,
        origin=Origin(xyz=(0.058, 0.010, -0.040)),
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="handle_hub",
    )
    handle.visual(
        Box((0.020, 0.012, 0.058)),
        origin=Origin(xyz=(0.020, 0.010, -0.024)),
        material=handle_metal,
        name="handle_neck",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.092),
        origin=Origin(xyz=(0.030, 0.010, -0.080)),
        material=handle_metal,
        name="handle_grip",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.5 * opening_width, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "sash_to_handle",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(xyz=(handle_pivot_x, 0.009, handle_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-1.1, upper=0.35),
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

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    handle = object_model.get_part("handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    handle_joint = object_model.get_articulation("sash_to_handle")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.check("frame part exists", frame is not None, details="frame part missing")
    ctx.check("sash part exists", sash is not None, details="sash part missing")
    ctx.check("handle part exists", handle is not None, details="handle part missing")
    ctx.check(
        "sash hinge uses vertical axis",
        tuple(round(v, 6) for v in sash_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={sash_hinge.axis}",
    )
    ctx.check(
        "handle pivot uses transverse axis",
        tuple(round(v, 6) for v in handle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )

    with ctx.pose({sash_hinge: 0.0, handle_joint: 0.0}):
        ctx.expect_gap(
            sash,
            frame,
            axis="x",
            positive_elem="sash_left_stile",
            negative_elem="frame_left_jamb",
            min_gap=0.003,
            max_gap=0.006,
            name="hinge-side reveal stays narrow",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="frame_right_jamb",
            negative_elem="sash_right_stile",
            min_gap=0.003,
            max_gap=0.006,
            name="latch-side reveal stays narrow",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="frame_head",
            negative_elem="sash_top_rail",
            min_gap=0.003,
            max_gap=0.006,
            name="head reveal stays narrow",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="frame_sill",
            min_gap=0.003,
            max_gap=0.006,
            name="sill reveal stays narrow",
        )
        ctx.expect_contact(
            handle,
            sash,
            elem_a="handle_hub",
            elem_b="handle_base_boss",
            contact_tol=0.0005,
            name="handle hub bears on its base boss",
        )

        closed_right_stile = _center_from_aabb(ctx.part_element_world_aabb(sash, elem="sash_right_stile"))
        closed_handle_grip = _center_from_aabb(ctx.part_element_world_aabb(handle, elem="handle_grip"))

    with ctx.pose({sash_hinge: 0.85, handle_joint: 0.0}):
        opened_right_stile = _center_from_aabb(ctx.part_element_world_aabb(sash, elem="sash_right_stile"))

    ctx.check(
        "sash swings outward from hinge jamb",
        closed_right_stile is not None
        and opened_right_stile is not None
        and opened_right_stile[1] > closed_right_stile[1] + 0.25,
        details=f"closed={closed_right_stile}, opened={opened_right_stile}",
    )

    with ctx.pose({sash_hinge: 0.0, handle_joint: -0.9}):
        turned_handle_grip = _center_from_aabb(ctx.part_element_world_aabb(handle, elem="handle_grip"))

    ctx.check(
        "lever handle rotates about its own pivot",
        closed_handle_grip is not None
        and turned_handle_grip is not None
        and turned_handle_grip[2] > closed_handle_grip[2] + 0.05,
        details=f"closed={closed_handle_grip}, turned={turned_handle_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
