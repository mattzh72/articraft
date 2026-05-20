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


def _wheel_part(model: ArticulatedObject, name: str, tire_mat, core_mat, hub_mat):
    wheel = model.part(name)
    tire_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.052, -0.012), (0.055, 0.0), (0.052, 0.012)],
            inner_profile=[(0.047, -0.012), (0.047, 0.0), (0.047, 0.012)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        f"{name}_tire",
    )
    core_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.047, -0.010), (0.047, 0.010)],
            inner_profile=[(0.021, -0.008), (0.021, 0.008)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        f"{name}_core",
    )
    hub_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.021, -0.014), (0.021, 0.014)],
            inner_profile=[(0.0095, -0.014), (0.0095, 0.014)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        f"{name}_hub",
    )
    wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=tire_mat,
        name="tire",
    )
    wheel.visual(
        core_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=core_mat,
        name="wheel_core",
    )
    wheel.visual(
        hub_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub",
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pro_stunt_scooter")

    deck_len = 0.52
    deck_width = 0.12
    deck_thickness = 0.024
    headtube_angle = math.radians(20.0)
    headtube_center = (0.255, 0.0, 0.085)
    headtube_radius = 0.034
    headtube_length = 0.16
    front_axle_xyz = (0.072, 0.0, -0.165)
    rear_axle_xyz = (-0.22, 0.0, -0.069)
    fold_hinge_z = 0.139

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    raw_aluminum = model.material("raw_aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.08, 0.08, 0.08, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.80, 0.82, 0.86, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.44, deck_width, deck_thickness)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        material=matte_black,
        name="deck_shell",
    )
    deck.visual(
        Box((0.164, 0.016, 0.034)),
        origin=Origin(xyz=(0.188, 0.035, 0.031), rpy=(0.0, -0.58, 0.0)),
        material=matte_black,
        name="left_neck_gusset",
    )
    deck.visual(
        Box((0.164, 0.016, 0.034)),
        origin=Origin(xyz=(0.188, -0.035, 0.031), rpy=(0.0, -0.58, 0.0)),
        material=matte_black,
        name="right_neck_gusset",
    )
    deck.visual(
        Cylinder(radius=headtube_radius, length=headtube_length),
        origin=Origin(xyz=headtube_center, rpy=(0.0, -headtube_angle, 0.0)),
        material=raw_aluminum,
        name="headtube_shell",
    )
    deck.visual(
        Box((0.090, 0.096, 0.006)),
        origin=Origin(xyz=(-0.205, 0.0, 0.017), rpy=(0.0, 0.22, 0.0)),
        material=raw_aluminum,
        name="rear_brake_fender",
    )
    deck.visual(
        Box((0.055, 0.008, 0.090)),
        origin=Origin(xyz=(-0.225, 0.046, -0.033)),
        material=matte_black,
        name="left_dropout",
    )
    deck.visual(
        Box((0.055, 0.008, 0.090)),
        origin=Origin(xyz=(-0.225, -0.046, -0.033)),
        material=matte_black,
        name="right_dropout",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.108),
        origin=Origin(xyz=rear_axle_xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=raw_aluminum,
        name="rear_axle",
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.024, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=raw_aluminum,
        name="steerer_tube",
    )
    fork.visual(
        Box((0.052, 0.058, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=raw_aluminum,
        name="fork_clamp",
    )
    fork.visual(
        Box((0.044, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.019, 0.129)),
        material=raw_aluminum,
        name="left_hinge_ear",
    )
    fork.visual(
        Box((0.044, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.019, 0.129)),
        material=raw_aluminum,
        name="right_hinge_ear",
    )
    fork.visual(
        Box((0.080, 0.088, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, -0.092)),
        material=raw_aluminum,
        name="fork_crown",
    )
    fork.visual(
        Box((0.060, 0.010, 0.112)),
        origin=Origin(xyz=(0.045, 0.038, -0.148)),
        material=raw_aluminum,
        name="left_fork_leg",
    )
    fork.visual(
        Box((0.060, 0.010, 0.112)),
        origin=Origin(xyz=(0.045, -0.038, -0.148)),
        material=raw_aluminum,
        name="right_fork_leg",
    )
    fork.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=front_axle_xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=raw_aluminum,
        name="front_axle",
    )

    stem = model.part("stem")
    stem.visual(
        Box((0.044, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=raw_aluminum,
        name="stem_hinge_block",
    )
    stem.visual(
        Cylinder(radius=0.021, length=0.676),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=matte_black,
        name="stem_tube",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="handlebar_bar",
    )
    stem.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(0.0, 0.280, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(0.0, -0.280, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    front_wheel = _wheel_part(model, "front_wheel", wheel_urethane, wheel_core, raw_aluminum)
    rear_wheel = _wheel_part(model, "rear_wheel", wheel_urethane, wheel_core, raw_aluminum)

    model.articulation(
        "deck_to_fork_steer",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=fork,
        origin=Origin(xyz=headtube_center, rpy=(0.0, -headtube_angle, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "fork_to_stem_fold",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, fold_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "fork_to_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_wheel,
        origin=Origin(xyz=front_axle_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )
    model.articulation(
        "deck_to_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
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

    deck = object_model.get_part("deck")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    steer = object_model.get_articulation("deck_to_fork_steer")
    fold = object_model.get_articulation("fork_to_stem_fold")
    front_spin = object_model.get_articulation("fork_to_front_wheel_spin")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel_spin")

    ctx.allow_overlap(
        deck,
        fork,
        elem_a="headtube_shell",
        elem_b="steerer_tube",
        reason="The deck headtube is represented as a solid outer shell around the fork steerer and headset stack.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="hub",
        reason="The rear axle intentionally passes through the wheel hub.",
    )
    ctx.allow_overlap(
        fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="hub",
        reason="The front axle intentionally passes through the wheel hub.",
    )

    ctx.check(
        "all scooter parts exist",
        all(part is not None for part in (deck, fork, stem, front_wheel, rear_wheel)),
        details="Expected deck, fork, stem, front wheel, and rear wheel parts.",
    )
    ctx.check(
        "articulation types match scooter mechanisms",
        steer.articulation_type == ArticulationType.CONTINUOUS
        and fold.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"steer={steer.articulation_type}, fold={fold.articulation_type}, "
            f"front_spin={front_spin.articulation_type}, rear_spin={rear_spin.articulation_type}"
        ),
    )
    ctx.check(
        "joint axes follow scooter layout",
        steer.axis == (0.0, 0.0, 1.0)
        and fold.axis == (0.0, -1.0, 0.0)
        and front_spin.axis == (0.0, 1.0, 0.0)
        and rear_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"steer.axis={steer.axis}, fold.axis={fold.axis}, "
            f"front_spin.axis={front_spin.axis}, rear_spin.axis={rear_spin.axis}"
        ),
    )
    ctx.expect_contact(
        stem,
        fork,
        elem_a="stem_hinge_block",
        elem_b="left_hinge_ear",
        name="stem hinge tongue seats between the fork hinge ears",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="x",
        elem_a="tire",
        elem_b="deck_shell",
        min_overlap=0.020,
        name="rear wheel sits under the deck tail",
    )

    rest_front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.60}):
        steered_front_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork steering swings the wheel sideways",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[1]) > abs(rest_front_pos[1]) + 0.020,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))

    rest_bar_center = _aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
    with ctx.pose({fold: 1.10}):
        folded_bar_center = _aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
    ctx.check(
        "folding stem drops and tucks the bars rearward",
        rest_bar_center is not None
        and folded_bar_center is not None
        and folded_bar_center[0] < rest_bar_center[0] - 0.120
        and folded_bar_center[2] < rest_bar_center[2] - 0.180,
        details=f"rest_bar_center={rest_bar_center}, folded_bar_center={folded_bar_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
