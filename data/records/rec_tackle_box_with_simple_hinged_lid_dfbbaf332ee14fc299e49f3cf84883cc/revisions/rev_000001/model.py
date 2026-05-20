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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tackle_box")

    shell_color = model.material("shell_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    lid_color = model.material("lid_gunmetal", rgba=(0.26, 0.28, 0.31, 1.0))
    hardware_color = model.material("hardware_black", rgba=(0.09, 0.10, 0.11, 1.0))
    datum_color = model.material("datum_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    mark_color = model.material("index_white", rgba=(0.93, 0.94, 0.95, 1.0))
    seal_color = model.material("seal_blue", rgba=(0.18, 0.34, 0.56, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((0.440, 0.280, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=shell_color,
        name="bottom_pan",
    )
    shell.visual(
        Box((0.440, 0.005, 0.114)),
        origin=Origin(xyz=(0.000, 0.1375, 0.057)),
        material=shell_color,
        name="front_wall",
    )
    shell.visual(
        Box((0.440, 0.005, 0.114)),
        origin=Origin(xyz=(0.000, -0.1375, 0.057)),
        material=shell_color,
        name="rear_wall",
    )
    shell.visual(
        Box((0.005, 0.270, 0.114)),
        origin=Origin(xyz=(-0.2175, 0.000, 0.057)),
        material=shell_color,
        name="left_wall",
    )
    shell.visual(
        Box((0.005, 0.270, 0.114)),
        origin=Origin(xyz=(0.2175, 0.000, 0.057)),
        material=shell_color,
        name="right_wall",
    )
    shell.visual(
        Box((0.420, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, 0.130, 0.116)),
        material=datum_color,
        name="front_rim_land",
    )
    shell.visual(
        Box((0.012, 0.254, 0.008)),
        origin=Origin(xyz=(-0.214, 0.000, 0.116)),
        material=datum_color,
        name="left_rim_land",
    )
    shell.visual(
        Box((0.012, 0.254, 0.008)),
        origin=Origin(xyz=(0.214, 0.000, 0.116)),
        material=datum_color,
        name="right_rim_land",
    )
    shell.visual(
        Box((0.180, 0.004, 0.050)),
        origin=Origin(xyz=(0.000, -0.020, 0.025)),
        material=shell_color,
        name="center_divider",
    )
    shell.visual(
        Box((0.004, 0.120, 0.050)),
        origin=Origin(xyz=(-0.080, 0.045, 0.025)),
        material=shell_color,
        name="left_compartment_divider",
    )
    shell.visual(
        Box((0.004, 0.120, 0.050)),
        origin=Origin(xyz=(0.080, 0.045, 0.025)),
        material=shell_color,
        name="right_compartment_divider",
    )
    shell.visual(
        Box((0.230, 0.010, 0.012)),
        origin=Origin(xyz=(0.000, -0.133, 0.104)),
        material=shell_color,
        name="hinge_bridge",
    )
    shell.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(-0.158, -0.145, 0.114), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="left_hinge_barrel",
    )
    shell.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(0.000, -0.145, 0.114), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="center_hinge_barrel",
    )
    shell.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(0.158, -0.145, 0.114), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="right_hinge_barrel",
    )
    shell.visual(
        Box((0.048, 0.006, 0.018)),
        origin=Origin(xyz=(-0.140, 0.139, 0.059)),
        material=hardware_color,
        name="left_latch_mount",
    )
    shell.visual(
        Box((0.048, 0.006, 0.018)),
        origin=Origin(xyz=(0.140, 0.139, 0.059)),
        material=hardware_color,
        name="right_latch_mount",
    )
    shell.visual(
        Box((0.038, 0.006, 0.008)),
        origin=Origin(xyz=(-0.140, 0.143, 0.060)),
        material=datum_color,
        name="left_latch_pad",
    )
    shell.visual(
        Box((0.038, 0.006, 0.008)),
        origin=Origin(xyz=(0.140, 0.143, 0.060)),
        material=datum_color,
        name="right_latch_pad",
    )
    shell.visual(
        Box((0.090, 0.006, 0.020)),
        origin=Origin(xyz=(0.000, 0.143, 0.100)),
        material=datum_color,
        name="front_datum_band",
    )
    shell.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(-0.160, -0.100, 0.003)),
        material=hardware_color,
        name="left_rear_foot",
    )
    shell.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(0.160, -0.100, 0.003)),
        material=hardware_color,
        name="right_rear_foot",
    )
    shell.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(-0.160, 0.100, 0.003)),
        material=hardware_color,
        name="left_front_foot",
    )
    shell.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(0.160, 0.100, 0.003)),
        material=hardware_color,
        name="right_front_foot",
    )
    shell.inertial = Inertial.from_geometry(
        Box((0.450, 0.290, 0.125)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.0625)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.450, 0.290, 0.006)),
        origin=Origin(xyz=(0.000, 0.145, 0.0065)),
        material=lid_color,
        name="top_panel",
    )
    lid.visual(
        Box((0.004, 0.286, 0.028)),
        origin=Origin(xyz=(-0.223, 0.143, -0.0105)),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((0.004, 0.286, 0.028)),
        origin=Origin(xyz=(0.223, 0.143, -0.0105)),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((0.442, 0.004, 0.028)),
        origin=Origin(xyz=(0.000, 0.288, -0.0105)),
        material=lid_color,
        name="front_skirt",
    )
    lid.visual(
        Box((0.320, 0.090, 0.002)),
        origin=Origin(xyz=(0.000, 0.148, 0.0105)),
        material=datum_color,
        name="datum_plate",
    )
    lid.visual(
        Box((0.180, 0.004, 0.001)),
        origin=Origin(xyz=(0.000, 0.148, 0.0120)),
        material=mark_color,
        name="x_index_mark",
    )
    lid.visual(
        Box((0.004, 0.060, 0.001)),
        origin=Origin(xyz=(0.000, 0.148, 0.0120)),
        material=mark_color,
        name="y_index_mark",
    )
    lid.visual(
        Box((0.030, 0.004, 0.001)),
        origin=Origin(xyz=(-0.105, 0.148, 0.0120)),
        material=mark_color,
        name="left_tick_mark",
    )
    lid.visual(
        Box((0.030, 0.004, 0.001)),
        origin=Origin(xyz=(0.105, 0.148, 0.0120)),
        material=mark_color,
        name="right_tick_mark",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.155, 0.090, 0.0115)),
        material=datum_color,
        name="left_adjuster",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.155, 0.090, 0.0115)),
        material=datum_color,
        name="right_adjuster",
    )
    lid.visual(
        Box((0.012, 0.002, 0.001)),
        origin=Origin(xyz=(-0.155, 0.090, 0.0140)),
        material=hardware_color,
        name="left_adjuster_slot",
    )
    lid.visual(
        Box((0.012, 0.002, 0.001)),
        origin=Origin(xyz=(0.155, 0.090, 0.0140)),
        material=hardware_color,
        name="right_adjuster_slot",
    )
    lid.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(-0.140, 0.294, -0.014)),
        material=datum_color,
        name="left_keeper",
    )
    lid.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.140, 0.294, -0.014)),
        material=datum_color,
        name="right_keeper",
    )
    lid.visual(
        Box((0.018, 0.024, 0.025)),
        origin=Origin(xyz=(-0.080, 0.145, 0.0220)),
        material=hardware_color,
        name="left_handle_pedestal",
    )
    lid.visual(
        Box((0.018, 0.024, 0.025)),
        origin=Origin(xyz=(0.080, 0.145, 0.0220)),
        material=hardware_color,
        name="right_handle_pedestal",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(0.000, 0.145, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="carry_handle",
    )
    lid.visual(
        Box((0.280, 0.010, 0.0015)),
        origin=Origin(xyz=(0.000, 0.145, 0.0038)),
        material=seal_color,
        name="seal_strip",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(-0.079, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="left_lid_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.079, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="right_lid_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.450, 0.290, 0.060)),
        mass=1.1,
        origin=Origin(xyz=(0.000, 0.145, 0.010)),
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.000, -0.145, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    def add_front_latch(name: str, x_pos: float) -> None:
        latch = model.part(name)
        latch.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware_color,
            name="barrel",
        )
        latch.visual(
            Box((0.026, 0.010, 0.024)),
            origin=Origin(xyz=(0.000, 0.001, 0.012)),
            material=hardware_color,
            name="arm",
        )
        latch.visual(
            Box((0.028, 0.018, 0.012)),
            origin=Origin(xyz=(0.000, 0.006, 0.008)),
            material=hardware_color,
            name="grip",
        )
        latch.visual(
            Box((0.022, 0.004, 0.008)),
            origin=Origin(xyz=(0.000, -0.006, 0.032)),
            material=datum_color,
            name="hook",
        )
        latch.visual(
            Box((0.010, 0.006, 0.014)),
            origin=Origin(xyz=(0.000, -0.005, 0.025)),
            material=hardware_color,
            name="hook_stem",
        )
        latch.inertial = Inertial.from_geometry(
            Box((0.030, 0.018, 0.050)),
            mass=0.06,
            origin=Origin(xyz=(0.000, 0.002, 0.021)),
        )
        model.articulation(
            f"shell_to_{name}",
            ArticulationType.REVOLUTE,
            parent=shell,
            child=latch,
            origin=Origin(xyz=(x_pos, 0.149, 0.074)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=-1.10,
                upper=0.0,
            ),
        )

    add_front_latch("left_latch", -0.140)
    add_front_latch("right_latch", 0.140)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    left_latch_joint = object_model.get_articulation("shell_to_left_latch")
    right_latch_joint = object_model.get_articulation("shell_to_right_latch")

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
        "explicit_primary_articulations",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and left_latch_joint.axis == (1.0, 0.0, 0.0)
        and right_latch_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"lid axis={lid_hinge.axis}, "
            f"left latch axis={left_latch_joint.axis}, "
            f"right latch axis={right_latch_joint.axis}"
        ),
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="top_panel",
        negative_elem="front_rim_land",
        min_gap=0.0010,
        max_gap=0.0030,
        name="controlled_closed_gap_above_front_rim",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        min_overlap=0.220,
        elem_a="top_panel",
        elem_b="bottom_pan",
        name="lid_covers_shell_planform",
    )
    ctx.expect_gap(
        lid,
        left_latch,
        axis="y",
        positive_elem="left_keeper",
        negative_elem="hook",
        max_gap=0.0010,
        max_penetration=0.0,
        name="left_latch_closes_to_keeper",
    )
    ctx.expect_gap(
        lid,
        right_latch,
        axis="y",
        positive_elem="right_keeper",
        negative_elem="hook",
        max_gap=0.0010,
        max_penetration=0.0,
        name="right_latch_closes_to_keeper",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    closed_front = _aabb_center(ctx.part_element_world_aabb(lid, elem="front_skirt"))
    with ctx.pose(
        {
            lid_hinge: 1.20,
            left_latch_joint: -1.00,
            right_latch_joint: -1.00,
        }
    ):
        opened_front = _aabb_center(ctx.part_element_world_aabb(lid, elem="front_skirt"))
        ctx.fail_if_parts_overlap_in_current_pose(name="released_open_pose_clearance")

    if closed_front is None or opened_front is None:
        ctx.fail("lid_front_pose_tracking", "could not measure front skirt in closed/open poses")
    else:
        ctx.check(
            "lid_front_rises_when_opened",
            opened_front[2] > closed_front[2] + 0.120,
            details=f"closed_z={closed_front[2]:.4f}, open_z={opened_front[2]:.4f}",
        )
        ctx.check(
            "lid_front_swings_clear_of_front_datum",
            opened_front[1] < closed_front[1] - 0.120,
            details=f"closed_y={closed_front[1]:.4f}, open_y={opened_front[1]:.4f}",
        )

    closed_hook = _aabb_center(ctx.part_element_world_aabb(left_latch, elem="hook"))
    with ctx.pose({left_latch_joint: -1.00}):
        open_hook = _aabb_center(ctx.part_element_world_aabb(left_latch, elem="hook"))
    if closed_hook is None or open_hook is None:
        ctx.fail("left_latch_pose_tracking", "could not measure left latch hook")
    else:
        ctx.check(
            "left_latch_releases_forward",
            open_hook[1] > closed_hook[1] + 0.025,
            details=f"closed_hook_y={closed_hook[1]:.4f}, open_hook_y={open_hook[1]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
