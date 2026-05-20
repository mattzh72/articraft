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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender_wand")

    cup_clear = model.material("cup_clear", rgba=(0.82, 0.88, 0.92, 0.45))
    socket_metal = model.material("socket_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    body_black = model.material("body_black", rgba=(0.14, 0.15, 0.17, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    base_pad = model.material("base_pad", rgba=(0.16, 0.16, 0.18, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    cup_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.0),
            (0.028, 0.003),
            (0.047, 0.012),
            (0.048, 0.090),
            (0.046, 0.138),
            (0.043, 0.148),
        ],
        [
            (0.0, 0.006),
            (0.040, 0.014),
            (0.041, 0.090),
            (0.039, 0.135),
            (0.036, 0.143),
        ],
        segments=72,
        lip_samples=8,
    )
    bell_guard = LatheGeometry.from_shell_profiles(
        [
            (0.010, 0.0),
            (0.012, 0.008),
            (0.015, 0.018),
            (0.018, 0.032),
            (0.017, 0.042),
            (0.014, 0.050),
        ],
        [
            (0.007, 0.0),
            (0.009, 0.008),
            (0.011, 0.018),
            (0.014, 0.032),
            (0.013, 0.042),
        ],
        segments=64,
        lip_samples=6,
    )
    base_cup = model.part("base_cup")
    base_cup.visual(_mesh("cup_shell", cup_shell), material=cup_clear, name="cup_shell")
    base_cup.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=base_pad,
        name="base_pad",
    )
    base_cup.visual(
        Box((0.002, 0.014, 0.022)),
        origin=Origin(xyz=(0.027, 0.0, 0.147)),
        material=socket_metal,
        name="socket_guide_right",
    )
    base_cup.visual(
        Box((0.012, 0.004, 0.022)),
        origin=Origin(xyz=(0.034, 0.0, 0.147)),
        material=socket_metal,
        name="socket_rib_right",
    )
    base_cup.visual(
        Box((0.002, 0.014, 0.022)),
        origin=Origin(xyz=(-0.027, 0.0, 0.147)),
        material=socket_metal,
        name="socket_guide_left",
    )
    base_cup.visual(
        Box((0.012, 0.004, 0.022)),
        origin=Origin(xyz=(-0.034, 0.0, 0.147)),
        material=socket_metal,
        name="socket_rib_left",
    )
    base_cup.visual(
        Box((0.014, 0.002, 0.022)),
        origin=Origin(xyz=(0.0, 0.027, 0.147)),
        material=socket_metal,
        name="socket_guide_front",
    )
    base_cup.visual(
        Box((0.004, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.034, 0.147)),
        material=socket_metal,
        name="socket_rib_front",
    )
    base_cup.visual(
        Box((0.014, 0.002, 0.022)),
        origin=Origin(xyz=(0.0, -0.027, 0.147)),
        material=socket_metal,
        name="socket_guide_back",
    )
    base_cup.visual(
        Box((0.004, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.034, 0.147)),
        material=socket_metal,
        name="socket_rib_back",
    )
    base_cup.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(0.034, 0.0, 0.148)),
        material=socket_metal,
        name="bayonet_lug_right",
    )
    base_cup.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(-0.034, 0.0, 0.148)),
        material=socket_metal,
        name="bayonet_lug_left",
    )

    press_stage = model.part("press_stage")

    wand_body = model.part("wand_body")
    wand_body.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=socket_metal,
        name="lock_stub",
    )
    wand_body.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_black,
        name="lock_flange",
    )
    wand_body.visual(
        Cylinder(radius=0.028, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=body_black,
        name="motor_body",
    )
    wand_body.visual(
        Cylinder(radius=0.031, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=grip_gray,
        name="grip_sleeve",
    )
    wand_body.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.209)),
        material=body_black,
        name="top_cap",
    )
    wand_body.visual(
        Box((0.010, 0.006, 0.050)),
        origin=Origin(xyz=(0.029, 0.0, 0.112)),
        material=grip_gray,
        name="twist_fin",
    )
    wand_body.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.069)),
        material=socket_metal,
        name="drive_shaft",
    )
    wand_body.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        material=socket_metal,
        name="tip_collar",
    )
    wand_body.visual(
        _mesh("bell_guard", bell_guard),
        origin=Origin(xyz=(0.0, 0.0, -0.144)),
        material=socket_metal,
        name="tip_shroud",
    )

    blade_rotor = model.part("blade_rotor")
    blade_rotor.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=socket_metal,
        name="blade_hub",
    )
    blade_rotor.visual(
        Box((0.028, 0.005, 0.0016)),
        origin=Origin(
            xyz=(0.006, 0.0, -0.006),
            rpy=(0.28, 0.0, math.radians(18.0)),
        ),
        material=blade_steel,
        name="blade_primary",
    )
    blade_rotor.visual(
        Box((0.028, 0.005, 0.0016)),
        origin=Origin(
            xyz=(-0.006, 0.0, -0.006),
            rpy=(-0.28, 0.0, math.pi + math.radians(18.0)),
        ),
        material=blade_steel,
        name="blade_secondary",
    )

    model.articulation(
        "cup_to_press",
        ArticulationType.PRISMATIC,
        parent=base_cup,
        child=press_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=0.18,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "press_to_wand",
        ArticulationType.REVOLUTE,
        parent=press_stage,
        child=wand_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "wand_to_blade",
        ArticulationType.CONTINUOUS,
        parent=wand_body,
        child=blade_rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.124)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.5,
            velocity=35.0,
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

    base_cup = object_model.get_part("base_cup")
    press_stage = object_model.get_part("press_stage")
    wand_body = object_model.get_part("wand_body")
    blade_rotor = object_model.get_part("blade_rotor")

    press_joint = object_model.get_articulation("cup_to_press")
    twist_joint = object_model.get_articulation("press_to_wand")
    blade_joint = object_model.get_articulation("wand_to_blade")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (base_cup, press_stage, wand_body, blade_rotor)),
        details="One or more required parts could not be resolved.",
    )

    ctx.expect_overlap(
        wand_body,
        base_cup,
        axes="xy",
        min_overlap=0.030,
        name="wand stays centered over the cup",
    )
    ctx.expect_gap(
        wand_body,
        base_cup,
        axis="z",
        positive_elem="lock_flange",
        negative_elem="socket_guide_right",
        min_gap=0.006,
        max_gap=0.012,
        name="rest pose leaves a small press gap above the lock collar",
    )

    rest_wand_pos = ctx.part_world_position(wand_body)
    with ctx.pose({press_joint: 0.006}):
        pressed_wand_pos = ctx.part_world_position(wand_body)
        ctx.expect_gap(
            wand_body,
            base_cup,
            axis="z",
            positive_elem="lock_flange",
            negative_elem="socket_guide_right",
            max_gap=0.0015,
            max_penetration=0.0,
            name="pressed pose seats the lock flange onto the collar",
        )

    ctx.check(
        "press joint moves the wand downward",
        rest_wand_pos is not None
        and pressed_wand_pos is not None
        and pressed_wand_pos[2] < rest_wand_pos[2] - 0.0045,
        details=f"rest={rest_wand_pos}, pressed={pressed_wand_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    fin_rest = _aabb_center(ctx.part_element_world_aabb(wand_body, elem="twist_fin"))
    with ctx.pose({press_joint: 0.006, twist_joint: math.radians(38.0)}):
        fin_twisted = _aabb_center(ctx.part_element_world_aabb(wand_body, elem="twist_fin"))

    ctx.check(
        "twist lock rotates the wand body about the cup axis",
        fin_rest is not None
        and fin_twisted is not None
        and math.hypot(fin_twisted[0] - fin_rest[0], fin_twisted[1] - fin_rest[1]) > 0.012,
        details=f"rest={fin_rest}, twisted={fin_twisted}",
    )

    blade_rest = _aabb_center(ctx.part_element_world_aabb(blade_rotor, elem="blade_primary"))
    with ctx.pose({blade_joint: math.pi / 2.0}):
        blade_quarter_turn = _aabb_center(ctx.part_element_world_aabb(blade_rotor, elem="blade_primary"))

    ctx.check(
        "blade spins continuously at the wand tip",
        blade_rest is not None
        and blade_quarter_turn is not None
        and math.hypot(
            blade_quarter_turn[0] - blade_rest[0],
            blade_quarter_turn[1] - blade_rest[1],
        )
        > 0.008,
        details=f"rest={blade_rest}, quarter_turn={blade_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
