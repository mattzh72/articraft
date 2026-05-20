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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    body_white = model.material("body_white", rgba=(0.90, 0.90, 0.87, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    bowl_clear = model.material("bowl_clear", rgba=(0.78, 0.88, 0.92, 0.42))
    lid_clear = model.material("lid_clear", rgba=(0.82, 0.90, 0.95, 0.38))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        _mesh(
            "base_lower_body",
            ExtrudeGeometry(rounded_rect_profile(0.34, 0.24, 0.032), 0.060),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_white,
        name="lower_body",
    )
    base.visual(
        _mesh(
            "base_upper_body",
            ExtrudeGeometry(rounded_rect_profile(0.22, 0.18, 0.028), 0.070),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=body_white,
        name="upper_body",
    )
    base.visual(
        Box((0.128, 0.008, 0.038)),
        origin=Origin(xyz=(0.0, -0.116, 0.064)),
        material=trim_graphite,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, -0.121, 0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_silver,
        name="control_dial",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=trim_graphite,
        name="drive_collar",
    )
    for index, (x_sign, y_sign) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        base.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(0.118 * x_sign, 0.078 * y_sign, 0.006),
            ),
            material=rubber_black,
            name=f"foot_{index + 1}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.24, 0.14)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.0),
            (0.082, 0.0),
            (0.102, 0.006),
            (0.112, 0.024),
            (0.116, 0.112),
            (0.118, 0.152),
            (0.121, 0.164),
        ],
        [
            (0.011, 0.004),
            (0.070, 0.004),
            (0.094, 0.010),
            (0.104, 0.026),
            (0.108, 0.112),
            (0.112, 0.157),
        ],
        segments=64,
    )
    bowl.visual(
        _mesh("bowl_shell", bowl_shell),
        material=bowl_clear,
        name="bowl_shell",
    )
    bowl.visual(
        _mesh(
            "bowl_support_ring",
            LatheGeometry.from_shell_profiles(
                [
                    (0.013, 0.0),
                    (0.082, 0.0),
                    (0.082, 0.010),
                    (0.013, 0.010),
                ],
                [
                    (0.019, 0.002),
                    (0.076, 0.002),
                    (0.076, 0.008),
                    (0.019, 0.008),
                ],
                segments=48,
            ),
        ),
        material=bowl_clear,
        name="bowl_support_ring",
    )
    bowl.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_graphite,
        name="drive_post",
    )
    bowl.visual(
        Box((0.018, 0.032, 0.110)),
        origin=Origin(xyz=(0.146, 0.0, 0.094)),
        material=bowl_clear,
        name="handle_grip",
    )
    bowl.visual(
        Box((0.028, 0.028, 0.030)),
        origin=Origin(xyz=(0.130, 0.0, 0.136)),
        material=bowl_clear,
        name="handle_upper_mount",
    )
    bowl.visual(
        Box((0.028, 0.028, 0.030)),
        origin=Origin(xyz=(0.130, 0.0, 0.048)),
        material=bowl_clear,
        name="handle_lower_mount",
    )
    bowl.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.079, 0.010)),
        material=bowl_clear,
        name="bayonet_lug_front",
    )
    bowl.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.079, 0.010)),
        material=bowl_clear,
        name="bayonet_lug_back",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(-0.1245, -0.023, 0.159),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bowl_clear,
        name="hinge_knuckle_front",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(-0.1245, 0.023, 0.159),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bowl_clear,
        name="hinge_knuckle_back",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.31, 0.26, 0.17)),
        mass=0.95,
        origin=Origin(xyz=(0.015, 0.0, 0.082)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_clear,
        name="lid_barrel",
    )
    lid.visual(
        Cylinder(radius=0.122, length=0.012),
        origin=Origin(xyz=(0.121, 0.0, 0.011)),
        material=lid_clear,
        name="lid_disc",
    )
    lid.visual(
        Box((0.014, 0.022, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, 0.012)),
        material=lid_clear,
        name="lid_hinge_bridge",
    )
    lid.visual(
        _mesh(
            "lid_feed_chute",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.070, 0.090, 0.012),
                [rounded_rect_profile(0.044, 0.064, 0.008)],
                height=0.090,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.151, 0.0, 0.062)),
        material=lid_clear,
        name="feed_chute",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.25, 0.25, 0.11)),
        mass=0.45,
        origin=Origin(xyz=(0.120, 0.0, 0.048)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.008, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=trim_graphite,
        name="blade_shaft",
    )
    blade.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=trim_graphite,
        name="blade_hub",
    )
    blade.visual(
        Box((0.104, 0.016, 0.004)),
        origin=Origin(
            xyz=(0.032, 0.0, 0.024),
            rpy=(0.0, 0.20, math.radians(18.0)),
        ),
        material=blade_steel,
        name="blade_arm_a",
    )
    blade.visual(
        Box((0.104, 0.016, 0.004)),
        origin=Origin(
            xyz=(-0.032, 0.0, 0.024),
            rpy=(0.0, -0.20, math.radians(198.0)),
        ),
        material=blade_steel,
        name="blade_arm_b",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.060),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_bowl_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.40,
            upper=0.0,
        ),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(-0.1245, 0.0, 0.159)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
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

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")

    bowl_lock = object_model.get_articulation("base_to_bowl_lock")
    lid_hinge = object_model.get_articulation("bowl_to_lid")
    blade_spin = object_model.get_articulation("bowl_to_blade")

    def elem_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    ctx.check(
        "food processor articulation types",
        bowl_lock.joint_type == ArticulationType.REVOLUTE
        and lid_hinge.joint_type == ArticulationType.REVOLUTE
        and blade_spin.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"bowl={bowl_lock.joint_type}, lid={lid_hinge.joint_type}, "
            f"blade={blade_spin.joint_type}"
        ),
    )
    ctx.check(
        "bowl lock and blade spin use vertical axes",
        bowl_lock.axis == (0.0, 0.0, 1.0) and blade_spin.axis == (0.0, 0.0, 1.0),
        details=f"bowl_axis={bowl_lock.axis}, blade_axis={blade_spin.axis}",
    )
    ctx.check(
        "lid hinge opens about bowl edge axis",
        lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"lid_axis={lid_hinge.axis}",
    )

    with ctx.pose({bowl_lock: 0.0, lid_hinge: 0.0}):
        ctx.expect_contact(
            bowl,
            base,
            elem_a="bowl_support_ring",
            elem_b="drive_collar",
            name="bowl support ring seats on drive collar",
        )
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            positive_elem="lid_disc",
            negative_elem="bowl_shell",
            max_gap=0.004,
            max_penetration=0.001,
            name="lid closes onto bowl rim height",
        )
        ctx.expect_overlap(
            lid,
            bowl,
            axes="xy",
            elem_a="lid_disc",
            elem_b="bowl_shell",
            min_overlap=0.20,
            name="lid covers bowl opening",
        )
        ctx.expect_within(
            blade,
            bowl,
            axes="xy",
            outer_elem="bowl_shell",
            margin=0.010,
            name="blade stays within bowl diameter",
        )
        ctx.expect_contact(
            blade,
            bowl,
            elem_a="blade_hub",
            elem_b="drive_post",
            name="blade hub seats on center drive post",
        )
        ctx.expect_overlap(
            blade,
            bowl,
            axes="z",
            elem_a="blade_shaft",
            elem_b="bowl_shell",
            min_overlap=0.040,
            name="blade occupies bowl working depth",
        )

    locked_lug_center = elem_center(
        ctx.part_element_world_aabb(bowl, elem="bayonet_lug_front")
    )
    with ctx.pose({bowl_lock: -0.32}):
        unlocked_lug_center = elem_center(
            ctx.part_element_world_aabb(bowl, elem="bayonet_lug_front")
        )
    ctx.check(
        "bowl bayonet joint twists the bowl",
        locked_lug_center is not None
        and unlocked_lug_center is not None
        and math.hypot(
            unlocked_lug_center[0] - locked_lug_center[0],
            unlocked_lug_center[1] - locked_lug_center[1],
        )
        > 0.020,
        details=f"locked={locked_lug_center}, unlocked={unlocked_lug_center}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(78.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on the hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.050,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    blade_rest_center = elem_center(ctx.part_element_world_aabb(blade, elem="blade_arm_a"))
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_turn_center = elem_center(ctx.part_element_world_aabb(blade, elem="blade_arm_a"))
        ctx.expect_within(
            blade,
            bowl,
            axes="xy",
            outer_elem="bowl_shell",
            margin=0.010,
            name="rotated blade remains within bowl",
        )
    ctx.check(
        "blade spins around the center post axis",
        blade_rest_center is not None
        and blade_turn_center is not None
        and math.hypot(
            blade_turn_center[0] - blade_rest_center[0],
            blade_turn_center[1] - blade_rest_center[1],
        )
        > 0.030,
        details=f"rest={blade_rest_center}, turned={blade_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
