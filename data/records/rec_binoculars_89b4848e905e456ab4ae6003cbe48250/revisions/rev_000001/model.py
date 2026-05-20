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


def _visual_center_from_aabb(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _build_objective_barrel(model: ArticulatedObject, name: str):
    barrel = model.part(name)
    barrel.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="mount_collar",
    )
    barrel.visual(
        Cylinder(radius=0.026, length=0.092),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="objective_bezel",
    )
    barrel.visual(
        Cylinder(radius=0.022, length=0.002),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="objective_glass",
        name="objective_lens",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.114, 0.060, 0.060)),
        mass=0.18,
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_vision_binoculars")

    model.material("housing_black", rgba=(0.12, 0.13, 0.12, 1.0))
    model.material("housing_olive", rgba=(0.23, 0.26, 0.20, 1.0))
    model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    model.material("hinge_metal", rgba=(0.32, 0.34, 0.35, 1.0))
    model.material("objective_glass", rgba=(0.20, 0.34, 0.38, 0.55))
    model.material("sensor_glass", rgba=(0.42, 0.12, 0.10, 0.60))

    bridge_body = model.part("bridge_body")
    bridge_body.visual(
        Box((0.084, 0.064, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="housing_olive",
        name="central_bridge_block",
    )
    bridge_body.visual(
        Box((0.054, 0.118, 0.044)),
        origin=Origin(xyz=(-0.056, 0.0, 0.004)),
        material="housing_olive",
        name="rear_eyepiece_housing",
    )
    bridge_body.visual(
        Box((0.096, 0.062, 0.028)),
        origin=Origin(xyz=(-0.002, 0.0, 0.040)),
        material="housing_olive",
        name="battery_housing",
    )
    bridge_body.visual(
        Cylinder(radius=0.014, length=0.048),
        origin=Origin(xyz=(0.004, 0.0, 0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="top_control_pod",
    )
    bridge_body.visual(
        Box((0.034, 0.028, 0.024)),
        origin=Origin(xyz=(0.032, 0.0, -0.018)),
        material="housing_black",
        name="lower_bridge_keel",
    )
    bridge_body.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.036, 0.046, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="left_barrel_socket",
    )
    bridge_body.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.036, -0.046, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="right_barrel_socket",
    )
    bridge_body.visual(
        Cylinder(radius=0.0175, length=0.028),
        origin=Origin(xyz=(-0.095, 0.032, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="left_eyepiece",
    )
    bridge_body.visual(
        Cylinder(radius=0.0175, length=0.028),
        origin=Origin(xyz=(-0.095, -0.032, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="right_eyepiece",
    )
    bridge_body.visual(
        Box((0.020, 0.050, 0.024)),
        origin=Origin(xyz=(-0.088, 0.0, 0.006)),
        material="housing_black",
        name="ocular_bridge_saddle",
    )
    bridge_body.visual(
        Box((0.018, 0.076, 0.012)),
        origin=Origin(xyz=(-0.076, 0.0, 0.024)),
        material="housing_black",
        name="hood_hinge_saddle",
    )
    bridge_body.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.073, 0.031, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="hood_hinge_knuckle_left",
    )
    bridge_body.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.073, -0.031, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="hood_hinge_knuckle_right",
    )
    bridge_body.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.051, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="ir_illuminator_housing",
    )
    bridge_body.visual(
        Box((0.010, 0.024, 0.006)),
        origin=Origin(xyz=(0.056, 0.0, -0.014)),
        material="housing_black",
        name="ir_cap_hinge_bridge",
    )
    bridge_body.visual(
        Cylinder(radius=0.003, length=0.006),
        origin=Origin(xyz=(0.060, 0.008, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="ir_cap_knuckle_left",
    )
    bridge_body.visual(
        Cylinder(radius=0.003, length=0.006),
        origin=Origin(xyz=(0.060, -0.008, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="ir_cap_knuckle_right",
    )
    bridge_body.visual(
        Cylinder(radius=0.0088, length=0.002),
        origin=Origin(xyz=(0.063, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="sensor_glass",
        name="ir_lens",
    )
    bridge_body.inertial = Inertial.from_geometry(
        Box((0.190, 0.150, 0.095)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    left_barrel = _build_objective_barrel(model, "left_objective_barrel")
    right_barrel = _build_objective_barrel(model, "right_objective_barrel")

    eyeshield_hood = model.part("eyeshield_hood")
    eyeshield_hood.visual(
        Box((0.050, 0.132, 0.003)),
        origin=Origin(xyz=(-0.025, 0.0, -0.0015)),
        material="rubber_black",
        name="hood_roof",
    )
    eyeshield_hood.visual(
        Box((0.050, 0.003, 0.038)),
        origin=Origin(xyz=(-0.025, 0.0645, -0.019)),
        material="rubber_black",
        name="hood_left_wall",
    )
    eyeshield_hood.visual(
        Box((0.050, 0.003, 0.038)),
        origin=Origin(xyz=(-0.025, -0.0645, -0.019)),
        material="rubber_black",
        name="hood_right_wall",
    )
    eyeshield_hood.visual(
        Box((0.003, 0.126, 0.020)),
        origin=Origin(xyz=(-0.0485, 0.0, -0.010)),
        material="rubber_black",
        name="hood_rear_lip",
    )
    eyeshield_hood.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="hood_center_knuckle",
    )
    eyeshield_hood.inertial = Inertial.from_geometry(
        Box((0.052, 0.134, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(-0.025, 0.0, -0.014)),
    )

    ir_illuminator_cap = model.part("ir_illuminator_cap")
    ir_illuminator_cap.visual(
        Box((0.006, 0.016, 0.006)),
        origin=Origin(xyz=(0.003, 0.0, -0.003)),
        material="hinge_metal",
        name="cap_hinge_tab",
    )
    ir_illuminator_cap.visual(
        Cylinder(radius=0.003, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_metal",
        name="cap_center_knuckle",
    )
    ir_illuminator_cap.visual(
        Cylinder(radius=0.013, length=0.003),
        origin=Origin(xyz=(0.006, 0.0, -0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="housing_black",
        name="cap_disc",
    )
    ir_illuminator_cap.visual(
        Cylinder(radius=0.0145, length=0.0015),
        origin=Origin(xyz=(0.0078, 0.0, -0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="cap_rim",
    )
    ir_illuminator_cap.inertial = Inertial.from_geometry(
        Box((0.016, 0.030, 0.030)),
        mass=0.02,
        origin=Origin(xyz=(0.006, 0.0, -0.013)),
    )

    model.articulation(
        "bridge_to_left_objective_barrel",
        ArticulationType.FIXED,
        parent=bridge_body,
        child=left_barrel,
        origin=Origin(xyz=(0.042, 0.046, -0.003)),
    )
    model.articulation(
        "bridge_to_right_objective_barrel",
        ArticulationType.FIXED,
        parent=bridge_body,
        child=right_barrel,
        origin=Origin(xyz=(0.042, -0.046, -0.003)),
    )
    model.articulation(
        "bridge_to_eyeshield_hood",
        ArticulationType.REVOLUTE,
        parent=bridge_body,
        child=eyeshield_hood,
        origin=Origin(xyz=(-0.073, 0.0, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "bridge_to_ir_illuminator_cap",
        ArticulationType.REVOLUTE,
        parent=bridge_body,
        child=ir_illuminator_cap,
        origin=Origin(xyz=(0.060, 0.0, -0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=1.50,
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

    body = object_model.get_part("bridge_body")
    left_barrel = object_model.get_part("left_objective_barrel")
    right_barrel = object_model.get_part("right_objective_barrel")
    hood = object_model.get_part("eyeshield_hood")
    cap = object_model.get_part("ir_illuminator_cap")
    hood_joint = object_model.get_articulation("bridge_to_eyeshield_hood")
    cap_joint = object_model.get_articulation("bridge_to_ir_illuminator_cap")

    ctx.check("bridge body exists", body is not None, details="bridge_body was not created")
    ctx.check("left objective barrel exists", left_barrel is not None, details="left barrel missing")
    ctx.check("right objective barrel exists", right_barrel is not None, details="right barrel missing")
    ctx.check("eyeshield hood exists", hood is not None, details="eyeshield hood missing")
    ctx.check("IR illuminator cap exists", cap is not None, details="IR illuminator cap missing")

    ctx.expect_contact(
        left_barrel,
        body,
        elem_a="mount_collar",
        elem_b="left_barrel_socket",
        name="left barrel is seated on the bridge socket",
    )
    ctx.expect_contact(
        right_barrel,
        body,
        elem_a="mount_collar",
        elem_b="right_barrel_socket",
        name="right barrel is seated on the bridge socket",
    )
    ctx.expect_origin_distance(
        left_barrel,
        right_barrel,
        axes="y",
        min_dist=0.088,
        max_dist=0.096,
        name="objective barrels keep binocular spacing",
    )

    with ctx.pose({hood_joint: 0.0, cap_joint: 0.0}):
        ctx.expect_overlap(
            cap,
            body,
            axes="yz",
            elem_a="cap_disc",
            elem_b="ir_lens",
            min_overlap=0.016,
            name="IR cap covers the illuminator lens when closed",
        )
        ctx.expect_gap(
            cap,
            body,
            axis="x",
            positive_elem="cap_disc",
            negative_elem="ir_lens",
            min_gap=0.0,
            max_gap=0.002,
            name="IR cap sits just ahead of the illuminator lens",
        )

        hood_lip_center = _visual_center_from_aabb(
            ctx.part_element_world_aabb(hood, elem="hood_rear_lip")
        )
        left_eyepiece_center = _visual_center_from_aabb(
            ctx.part_element_world_aabb(body, elem="left_eyepiece")
        )
        ctx.check(
            "hood shelters the eyepiece cluster when closed",
            hood_lip_center is not None
            and left_eyepiece_center is not None
            and hood_lip_center[0] < left_eyepiece_center[0] - 0.020
            and hood_lip_center[2] > left_eyepiece_center[2] + 0.015,
            details=f"hood_lip_center={hood_lip_center}, left_eyepiece_center={left_eyepiece_center}",
        )

        closed_hood_center = hood_lip_center
        closed_cap_center = _visual_center_from_aabb(ctx.part_element_world_aabb(cap, elem="cap_disc"))

    with ctx.pose({hood_joint: 1.15}):
        open_hood_center = _visual_center_from_aabb(ctx.part_element_world_aabb(hood, elem="hood_rear_lip"))
        ctx.check(
            "eyeshield hood flips upward",
            closed_hood_center is not None
            and open_hood_center is not None
            and open_hood_center[2] > closed_hood_center[2] + 0.025
            and open_hood_center[0] > closed_hood_center[0] + 0.015,
            details=f"closed={closed_hood_center}, open={open_hood_center}",
        )

    with ctx.pose({cap_joint: 1.35}):
        open_cap_center = _visual_center_from_aabb(ctx.part_element_world_aabb(cap, elem="cap_disc"))
        lens_center = _visual_center_from_aabb(ctx.part_element_world_aabb(body, elem="ir_lens"))
        ctx.check(
            "IR illuminator cap flips clear of the lens",
            closed_cap_center is not None
            and open_cap_center is not None
            and lens_center is not None
            and open_cap_center[2] > closed_cap_center[2] + 0.012
            and open_cap_center[2] > lens_center[2] + 0.010,
            details=f"closed={closed_cap_center}, open={open_cap_center}, lens={lens_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
