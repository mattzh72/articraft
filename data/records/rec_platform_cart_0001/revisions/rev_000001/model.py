from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

DECK_WIDTH = 0.50
DECK_LENGTH = 0.74
BOARD_THICKNESS = 0.028
BOARD_CENTER_Z = 0.178
CASTER_SWIVEL_Z = 0.150
WHEEL_RADIUS = 0.050
WHEEL_WIDTH = 0.024
WHEEL_AXLE_DROP = 0.100
HANDLE_HINGE_Y = 0.357
HANDLE_HINGE_Z = 0.170
HANDLE_FOLD_UPPER = 1.46

CASTER_LAYOUT = (
    ("front_left", 0.185, -0.285),
    ("front_right", -0.185, -0.285),
    ("rear_left", 0.185, 0.285),
    ("rear_right", -0.185, 0.285),
)


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba)


def _build_handle_frame_mesh():
    frame_path = [
        (-0.185, 0.016, 0.000),
        (-0.185, 0.024, 0.085),
        (-0.185, 0.040, 0.320),
        (-0.185, 0.070, 0.610),
        (-0.185, 0.095, 0.820),
        (0.185, 0.095, 0.820),
        (0.185, 0.070, 0.610),
        (0.185, 0.040, 0.320),
        (0.185, 0.024, 0.085),
        (0.185, 0.016, 0.000),
    ]
    frame_geom = tube_from_spline_points(
        frame_path,
        radius=0.011,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=False,
    )
    lower_crossbar = (
        CylinderGeometry(radius=0.012, height=0.390, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.018, 0.0)
    )
    left_hinge_collar = (
        CylinderGeometry(radius=0.017, height=0.028, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(-0.165, 0.010, 0.0)
    )
    right_hinge_collar = (
        CylinderGeometry(radius=0.017, height=0.028, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.165, 0.010, 0.0)
    )
    left_brace = BoxGeometry((0.028, 0.016, 0.060)).translate(-0.165, 0.020, 0.050)
    right_brace = BoxGeometry((0.028, 0.016, 0.060)).translate(0.165, 0.020, 0.050)
    frame_geom.merge(lower_crossbar)
    frame_geom.merge(left_hinge_collar)
    frame_geom.merge(right_hinge_collar)
    frame_geom.merge(left_brace)
    frame_geom.merge(right_brace)
    return mesh_from_geometry(frame_geom, ASSETS.mesh_path("platform_cart_handle_frame.obj"))


def _add_caster(
    model: ArticulatedObject,
    materials: dict[str, Material],
    corner_name: str,
    x_pos: float,
    y_pos: float,
) -> None:
    swivel = model.part(f"{corner_name}_swivel")
    swivel.visual(
        Box((0.070, 0.085, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=materials["zinc"],
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=materials["zinc"],
    )
    swivel.visual(
        Box((0.050, 0.045, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=materials["zinc"],
    )
    swivel.visual(
        Box((0.008, 0.046, 0.064)),
        origin=Origin(xyz=(0.029, 0.0, -0.078)),
        material=materials["zinc"],
    )
    swivel.visual(
        Box((0.008, 0.046, 0.064)),
        origin=Origin(xyz=(-0.029, 0.0, -0.078)),
        material=materials["zinc"],
    )
    swivel.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.020, 0.0, -0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["zinc"],
    )
    swivel.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(-0.020, 0.0, -0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["zinc"],
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.075, 0.085, 0.135)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    wheel = model.part(f"{corner_name}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["wheel"],
    )
    wheel.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["hub"],
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        f"{corner_name}_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent="deck",
        child=swivel.name,
        origin=Origin(xyz=(x_pos, y_pos, CASTER_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=8.0),
    )
    model.articulation(
        f"{corner_name}_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=swivel.name,
        child=wheel.name,
        origin=Origin(xyz=(0.0, 0.0, -WHEEL_AXLE_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=20.0),
    )


def build_object_model() -> ArticulatedObject:
    materials = {
        "deck": _material("sealed_wood", (0.47, 0.34, 0.22, 1.0)),
        "steel": _material("painted_steel", (0.20, 0.22, 0.24, 1.0)),
        "rubber": _material("rubber", (0.08, 0.08, 0.09, 1.0)),
        "zinc": _material("zinc_plated_steel", (0.70, 0.73, 0.76, 1.0)),
        "wheel": _material("polyurethane_wheel", (0.30, 0.32, 0.34, 1.0)),
        "hub": _material("machined_hub", (0.56, 0.58, 0.62, 1.0)),
        "handle": _material("powder_coat_handle", (0.74, 0.75, 0.77, 1.0)),
    }

    model = ArticulatedObject(name="platform_cart", assets=ASSETS)

    deck = model.part("deck")
    deck.visual(
        Box((DECK_WIDTH - 0.022, DECK_LENGTH - 0.020, BOARD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_CENTER_Z)),
        material=materials["deck"],
    )
    deck.visual(
        Box((DECK_WIDTH - 0.050, DECK_LENGTH - 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=materials["rubber"],
    )
    deck.visual(
        Box((0.018, DECK_LENGTH, 0.034)),
        origin=Origin(xyz=(0.241, 0.0, 0.176)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.018, DECK_LENGTH, 0.034)),
        origin=Origin(xyz=(-0.241, 0.0, 0.176)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.420, 0.020, 0.034)),
        origin=Origin(xyz=(0.0, -0.360, 0.176)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.036, 0.670, 0.032)),
        origin=Origin(xyz=(0.195, 0.0, 0.149)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.036, 0.670, 0.032)),
        origin=Origin(xyz=(-0.195, 0.0, 0.149)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.390, 0.036, 0.032)),
        origin=Origin(xyz=(0.0, -0.304, 0.149)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.390, 0.036, 0.032)),
        origin=Origin(xyz=(0.0, 0.304, 0.149)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.330, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, -0.105, 0.147)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.330, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.105, 0.147)),
        material=materials["steel"],
    )
    for caster_name, x_pos, y_pos in CASTER_LAYOUT:
        deck.visual(
            Box((0.085, 0.095, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.159)),
            material=materials["steel"],
            name=f"{caster_name}_mount_pad",
        )
    deck.visual(
        Box((0.380, 0.020, 0.034)),
        origin=Origin(xyz=(0.0, 0.347, HANDLE_HINGE_Z)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.060, 0.040, 0.038)),
        origin=Origin(xyz=(0.170, 0.334, 0.171)),
        material=materials["steel"],
    )
    deck.visual(
        Box((0.060, 0.040, 0.038)),
        origin=Origin(xyz=(-0.170, 0.334, 0.171)),
        material=materials["steel"],
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, DECK_LENGTH, 0.070)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    handle = model.part("handle")
    handle.visual(
        _build_handle_frame_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["handle"],
    )
    handle.visual(
        Cylinder(radius=0.017, length=0.250),
        origin=Origin(xyz=(0.0, 0.095, 0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.420, 0.090, 0.860)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.015, 0.410)),
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent="deck",
        child="handle",
        origin=Origin(xyz=(0.0, HANDLE_HINGE_Y, HANDLE_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.5,
            lower=0.0,
            upper=HANDLE_FOLD_UPPER,
        ),
    )

    for caster_name, x_pos, y_pos in CASTER_LAYOUT:
        _add_caster(model, materials, caster_name, x_pos, y_pos)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.02,
        reason="wheel axle centers sit inside open caster forks, so the visible parent metal stays slightly offset from the joint axis",
    )
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "deck",
        "handle",
        reason="folding handle hinge collars nest into the rear hinge bracket at the pivot",
    )
    for caster_name, _, _ in CASTER_LAYOUT:
        ctx.allow_overlap(
            "deck",
            f"{caster_name}_swivel",
            reason="caster top plate mounts tightly beneath the deck pad",
        )
        ctx.allow_overlap(
            f"{caster_name}_swivel",
            f"{caster_name}_wheel",
            reason="open fork geometry closely envelopes the wheel and the generated collision hulls conservatively report contact",
        )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=256, overlap_tol=0.004, overlap_volume_tol=0.0)

    for caster_name, _, _ in CASTER_LAYOUT:
        swivel_name = f"{caster_name}_swivel"
        wheel_name = f"{caster_name}_wheel"
        ctx.expect_aabb_overlap("deck", swivel_name, axes="xy", min_overlap=0.040)
        ctx.expect_aabb_overlap("deck", wheel_name, axes="xy", min_overlap=0.020)
        ctx.expect_aabb_gap("deck", wheel_name, axis="z", max_gap=0.055, max_penetration=0.0)
        ctx.expect_aabb_overlap(swivel_name, wheel_name, axes="xy", min_overlap=0.020)

    ctx.expect_aabb_overlap("handle", "deck", axes="xy", min_overlap=0.018)
    ctx.expect_joint_motion_axis(
        "handle_hinge",
        "handle",
        world_axis="z",
        direction="negative",
        min_delta=0.20,
    )

    with ctx.pose(handle_hinge=0.72):
        ctx.expect_aabb_overlap("handle", "deck", axes="xy", min_overlap=0.080)

    with ctx.pose(handle_hinge=HANDLE_FOLD_UPPER):
        ctx.expect_aabb_overlap("handle", "deck", axes="xy", min_overlap=0.140)

    with ctx.pose(
        front_left_caster_swivel=math.pi / 2.0,
        rear_right_caster_swivel=-math.pi / 2.0,
        front_left_wheel_roll=1.1,
        rear_right_wheel_roll=-0.8,
    ):
        ctx.expect_aabb_gap("deck", "front_left_wheel", axis="z", max_gap=0.055, max_penetration=0.0)
        ctx.expect_aabb_gap("deck", "rear_right_wheel", axis="z", max_gap=0.055, max_penetration=0.0)
        ctx.expect_aabb_overlap("deck", "front_left_wheel", axes="xy", min_overlap=0.020)
        ctx.expect_aabb_overlap("deck", "rear_right_wheel", axes="xy", min_overlap=0.020)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
