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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aircraft_access_hatch")

    frame_gray = model.material("frame_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.61, 0.64, 0.68, 1.0))
    steel = model.material("steel", rgba=(0.46, 0.49, 0.53, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.19, 0.20, 0.22, 1.0))

    frame_width = 0.62
    frame_height = 0.78
    frame_depth = 0.065
    opening_width = 0.46
    opening_height = 0.62
    hinge_axis_y = -0.247

    panel_outer_width = 0.434
    panel_outer_height = 0.594
    panel_ring_depth = 0.024
    panel_skin_width = 0.340
    panel_skin_height = 0.494
    panel_skin_depth = 0.016

    barrel_outer_radius = 0.017
    barrel_inner_radius = 0.0105

    def hinge_barrel_mesh(*, outer_radius: float, inner_radius: float, length: float, name: str):
        geom = LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -length * 0.5),
                (outer_radius, length * 0.5),
            ],
            [
                (inner_radius, -length * 0.5),
                (inner_radius, length * 0.5),
            ],
            segments=48,
        )
        return mesh_from_geometry(geom, name)

    frame = model.part("frame")
    frame.visual(
        Box((frame_depth, 0.080, 0.180)),
        origin=Origin(xyz=(0.0, -0.270, 0.300)),
        material=frame_gray,
        name="upper_left_jamb",
    )
    frame.visual(
        Box((frame_depth, 0.080, 0.180)),
        origin=Origin(xyz=(0.0, -0.270, -0.300)),
        material=frame_gray,
        name="lower_left_jamb",
    )
    frame.visual(
        Box((frame_depth, 0.080, frame_height)),
        origin=Origin(xyz=(0.0, 0.270, 0.0)),
        material=frame_gray,
        name="right_jamb",
    )
    frame.visual(
        Box((frame_depth, opening_width, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=frame_gray,
        name="top_rail",
    )
    frame.visual(
        Box((frame_depth, opening_width, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        material=frame_gray,
        name="bottom_rail",
    )
    frame.visual(
        hinge_barrel_mesh(
            outer_radius=barrel_outer_radius,
            inner_radius=barrel_inner_radius,
            length=0.160,
            name="upper_frame_hinge_barrel",
        ),
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.220)),
        material=steel,
        name="upper_frame_barrel",
    )
    frame.visual(
        hinge_barrel_mesh(
            outer_radius=barrel_outer_radius,
            inner_radius=barrel_inner_radius,
            length=0.160,
            name="lower_frame_hinge_barrel",
        ),
        origin=Origin(xyz=(0.0, hinge_axis_y, -0.220)),
        material=steel,
        name="lower_frame_barrel",
    )
    frame.visual(
        Cylinder(radius=barrel_inner_radius, length=0.600),
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.0)),
        material=dark_hardware,
        name="hinge_pin",
    )
    frame.visual(
        Box((0.020, 0.020, 0.040)),
        origin=Origin(xyz=(-0.010, 0.233, 0.245)),
        material=steel,
        name="upper_striker",
    )
    frame.visual(
        Box((0.020, 0.020, 0.040)),
        origin=Origin(xyz=(-0.010, 0.233, -0.245)),
        material=steel,
        name="lower_striker",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_depth, frame_width, frame_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    panel = model.part("panel")
    panel.visual(
        Box((panel_ring_depth, 0.048, panel_outer_height)),
        origin=Origin(xyz=(panel_ring_depth * 0.5, 0.054, 0.0)),
        material=panel_gray,
        name="hinge_stile",
    )
    panel.visual(
        Box((panel_ring_depth, 0.048, panel_outer_height)),
        origin=Origin(xyz=(panel_ring_depth * 0.5, 0.440, 0.0)),
        material=panel_gray,
        name="free_stile",
    )
    panel.visual(
        Box((panel_ring_depth, panel_outer_width - 0.096, 0.050)),
        origin=Origin(xyz=(panel_ring_depth * 0.5, 0.247, 0.272)),
        material=panel_gray,
        name="top_rail",
    )
    panel.visual(
        Box((panel_ring_depth, panel_outer_width - 0.096, 0.050)),
        origin=Origin(xyz=(panel_ring_depth * 0.5, 0.247, -0.272)),
        material=panel_gray,
        name="bottom_rail",
    )
    panel.visual(
        Box((panel_skin_depth, panel_skin_width, panel_skin_height)),
        origin=Origin(xyz=(panel_skin_depth * 0.5, 0.247, 0.0)),
        material=panel_gray,
        name="panel_inner_skin",
    )
    panel.visual(
        Box((0.010, 0.060, 0.276)),
        origin=Origin(xyz=(0.015, 0.023, 0.0)),
        material=steel,
        name="panel_hinge_leaf",
    )
    panel.visual(
        hinge_barrel_mesh(
            outer_radius=barrel_outer_radius,
            inner_radius=barrel_inner_radius,
            length=0.276,
            name="panel_hinge_barrel_mesh",
        ),
        material=steel,
        name="panel_hinge_barrel",
    )
    panel.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(0.029, 0.440, 0.245),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="upper_latch_boss",
    )
    panel.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(0.029, 0.440, -0.245),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lower_latch_boss",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.045, 0.452, 0.610)),
        mass=8.5,
        origin=Origin(xyz=(0.014, 0.247, 0.0)),
    )

    upper_latch_dog = model.part("upper_latch_dog")
    upper_latch_dog.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="hub",
    )
    upper_latch_dog.visual(
        Box((0.008, 0.042, 0.014)),
        origin=Origin(xyz=(0.006, 0.017, 0.0)),
        material=dark_hardware,
        name="arm",
    )
    upper_latch_dog.visual(
        Box((0.010, 0.014, 0.024)),
        origin=Origin(xyz=(0.007, 0.039, 0.009)),
        material=dark_hardware,
        name="toe",
    )
    upper_latch_dog.inertial = Inertial.from_geometry(
        Box((0.016, 0.052, 0.030)),
        mass=0.12,
        origin=Origin(xyz=(0.006, 0.018, 0.005)),
    )

    lower_latch_dog = model.part("lower_latch_dog")
    lower_latch_dog.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="hub",
    )
    lower_latch_dog.visual(
        Box((0.008, 0.042, 0.014)),
        origin=Origin(xyz=(0.006, 0.017, 0.0)),
        material=dark_hardware,
        name="arm",
    )
    lower_latch_dog.visual(
        Box((0.010, 0.014, 0.024)),
        origin=Origin(xyz=(0.007, 0.039, -0.009)),
        material=dark_hardware,
        name="toe",
    )
    lower_latch_dog.inertial = Inertial.from_geometry(
        Box((0.016, 0.052, 0.030)),
        mass=0.12,
        origin=Origin(xyz=(0.006, 0.018, -0.005)),
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "panel_to_upper_latch_dog",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=upper_latch_dog,
        origin=Origin(xyz=(0.038, 0.440, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.8,
            upper=0.8,
        ),
    )
    model.articulation(
        "panel_to_lower_latch_dog",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=lower_latch_dog,
        origin=Origin(xyz=(0.038, 0.440, -0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.8,
            upper=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    upper_latch_dog = object_model.get_part("upper_latch_dog")
    lower_latch_dog = object_model.get_part("lower_latch_dog")

    frame_to_panel = object_model.get_articulation("frame_to_panel")
    panel_to_upper_latch_dog = object_model.get_articulation("panel_to_upper_latch_dog")
    panel_to_lower_latch_dog = object_model.get_articulation("panel_to_lower_latch_dog")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        frame,
        panel,
        elem_a="hinge_pin",
        elem_b="panel_hinge_barrel",
        reason="The captive hinge pin runs inside the panel hinge barrel to keep the hatch supported by the jamb through the swing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        panel,
        frame,
        axes="xy",
        elem_a="panel_hinge_barrel",
        elem_b="upper_frame_barrel",
        min_overlap=0.02,
        name="panel hinge barrel aligns with upper hinge barrel",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="z",
        positive_elem="upper_frame_barrel",
        negative_elem="panel_hinge_barrel",
        min_gap=0.001,
        max_gap=0.003,
        name="upper hinge knuckle end gap is realistic",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="xy",
        elem_a="panel_hinge_barrel",
        elem_b="lower_frame_barrel",
        min_overlap=0.02,
        name="panel hinge barrel aligns with lower hinge barrel",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="panel_hinge_barrel",
        negative_elem="lower_frame_barrel",
        min_gap=0.001,
        max_gap=0.003,
        name="lower hinge knuckle end gap is realistic",
    )
    ctx.expect_contact(
        upper_latch_dog,
        panel,
        elem_a="hub",
        elem_b="upper_latch_boss",
        name="upper latch dog is mounted on panel boss",
    )
    ctx.expect_contact(
        lower_latch_dog,
        panel,
        elem_a="hub",
        elem_b="lower_latch_boss",
        name="lower latch dog is mounted on panel boss",
    )
    ctx.expect_within(
        panel,
        frame,
        axes="yz",
        margin=0.0,
        name="panel stays inside frame outline when closed",
    )

    ctx.check(
        "panel hinge axis is vertical",
        tuple(round(v, 6) for v in frame_to_panel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={frame_to_panel.axis}",
    )
    ctx.check(
        "latch dog pivots are normal to panel",
        tuple(round(v, 6) for v in panel_to_upper_latch_dog.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 6) for v in panel_to_lower_latch_dog.axis) == (1.0, 0.0, 0.0),
        details=(
            f"upper={panel_to_upper_latch_dog.axis}, "
            f"lower={panel_to_lower_latch_dog.axis}"
        ),
    )

    frame_aabb = ctx.part_world_aabb(frame)
    panel_skin_aabb = ctx.part_element_world_aabb(panel, elem="panel_inner_skin")
    if frame_aabb is not None and panel_skin_aabb is not None:
        frame_front_x = frame_aabb[1][0]
        panel_front_x = panel_skin_aabb[1][0]
        ctx.check(
            "panel face is recessed behind frame",
            0.012 <= frame_front_x - panel_front_x <= 0.020,
            details=(
                f"frame_front_x={frame_front_x:.4f}, "
                f"panel_front_x={panel_front_x:.4f}"
            ),
        )
    else:
        ctx.fail("panel face is recessed behind frame", "missing frame or panel AABB")

    upper_dog_aabb = ctx.part_world_aabb(upper_latch_dog)
    lower_dog_aabb = ctx.part_world_aabb(lower_latch_dog)
    if upper_dog_aabb is not None and lower_dog_aabb is not None:
        upper_center_z = (upper_dog_aabb[0][2] + upper_dog_aabb[1][2]) * 0.5
        lower_center_z = (lower_dog_aabb[0][2] + lower_dog_aabb[1][2]) * 0.5
        upper_front_edge_y = upper_dog_aabb[1][1]
        lower_front_edge_y = lower_dog_aabb[1][1]
        ctx.check(
            "latch dogs sit near top and bottom free-edge corners",
            upper_center_z > 0.18
            and lower_center_z < -0.18
            and upper_front_edge_y > 0.21
            and lower_front_edge_y > 0.21,
            details=(
                f"upper_z={upper_center_z:.4f}, lower_z={lower_center_z:.4f}, "
                f"upper_ymax={upper_front_edge_y:.4f}, lower_ymax={lower_front_edge_y:.4f}"
            ),
        )
    else:
        ctx.fail(
            "latch dogs sit near top and bottom free-edge corners",
            "missing latch dog AABBs",
        )

    panel_closed_aabb = ctx.part_element_world_aabb(panel, elem="panel_inner_skin")
    with ctx.pose({frame_to_panel: 1.1}):
        ctx.expect_overlap(
            panel,
            frame,
            axes="xy",
            elem_a="panel_hinge_barrel",
            elem_b="upper_frame_barrel",
            min_overlap=0.02,
            name="panel stays aligned to upper hinge barrel when open",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="upper_frame_barrel",
            negative_elem="panel_hinge_barrel",
            min_gap=0.001,
            max_gap=0.003,
            name="upper hinge end gap remains small when open",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xy",
            elem_a="panel_hinge_barrel",
            elem_b="lower_frame_barrel",
            min_overlap=0.02,
            name="panel stays aligned to lower hinge barrel when open",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="panel_hinge_barrel",
            negative_elem="lower_frame_barrel",
            min_gap=0.001,
            max_gap=0.003,
            name="lower hinge end gap remains small when open",
        )
        panel_open_aabb = ctx.part_element_world_aabb(panel, elem="panel_inner_skin")
        if panel_closed_aabb is not None and panel_open_aabb is not None:
            ctx.check(
                "panel swings outboard on the hinge",
                panel_open_aabb[0][0] < panel_closed_aabb[0][0] - 0.18,
                details=(
                    f"closed_min_x={panel_closed_aabb[0][0]:.4f}, "
                    f"open_min_x={panel_open_aabb[0][0]:.4f}"
                ),
            )
        else:
            ctx.fail("panel swings outboard on the hinge", "missing panel AABBs in pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
