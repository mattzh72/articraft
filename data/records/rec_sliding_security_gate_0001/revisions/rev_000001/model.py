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
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

CURB_LENGTH = 6.20
CURB_WIDTH = 0.70
CURB_CENTER_X = 1.50
MOUNT_Z = 0.12
TRACK_TOP_Z = 0.024
TRACK_LENGTH = 5.70
TRACK_CENTER_X = 1.65
TRACK_Y = 0.14
GATE_JOINT_X = -0.45
GATE_TRAVEL = 2.10


def make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in ({"name": name, "color": rgba}, {"name": name, "rgba": rgba}):
        try:
            return Material(**kwargs)
        except TypeError:
            continue
    return Material(name, rgba)


def add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material)


def add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def add_brace(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: float,
    depth: float,
    material: Material,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dz * dz)
    center = ((start[0] + end[0]) * 0.5, start[1], (start[2] + end[2]) * 0.5)
    pitch = -math.atan2(dz, dx)
    add_box(
        part,
        (length, depth, thickness),
        center,
        material,
        rpy=(0.0, pitch, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate", assets=ASSETS)

    concrete = make_material("concrete", (0.67, 0.67, 0.66, 1.0))
    powder_coat = make_material("powder_coat_charcoal", (0.19, 0.21, 0.23, 1.0))
    galvanized = make_material("galvanized_steel", (0.61, 0.63, 0.66, 1.0))
    stainless = make_material("brushed_stainless", (0.77, 0.79, 0.80, 1.0))
    rubber = make_material("black_rubber", (0.08, 0.08, 0.08, 1.0))
    amber = make_material("signal_amber", (0.95, 0.63, 0.16, 0.55))

    curb = model.part("curb_base")
    add_box(
        curb,
        (CURB_LENGTH, CURB_WIDTH, MOUNT_Z),
        (CURB_CENTER_X, 0.0, MOUNT_Z * 0.5),
        concrete,
    )
    curb.inertial = Inertial.from_geometry(
        Box((CURB_LENGTH, CURB_WIDTH, MOUNT_Z)),
        mass=860.0,
        origin=Origin(xyz=(CURB_CENTER_X, 0.0, MOUNT_Z * 0.5)),
    )

    track = model.part("ground_track")
    add_box(track, (TRACK_LENGTH, 0.11, 0.010), (0.0, 0.0, 0.005), galvanized)
    add_box(track, (TRACK_LENGTH, 0.026, 0.025), (0.0, 0.0, 0.0175), stainless)
    for x in (-2.45, -1.75, -1.05, -0.35, 0.35, 1.05, 1.75, 2.45):
        add_box(track, (0.070, 0.110, 0.004), (x, 0.0, 0.002), stainless)
    add_box(track, (0.080, 0.080, 0.020), (-2.78, 0.0, 0.010), galvanized)
    add_box(track, (0.080, 0.080, 0.020), (2.78, 0.0, 0.010), galvanized)
    track.inertial = Inertial.from_geometry(
        Box((TRACK_LENGTH, 0.11, 0.040)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    left_post = model.part("left_post")
    add_box(left_post, (0.18, 0.18, 2.35), (0.0, 0.0, 1.175), powder_coat)
    add_box(left_post, (0.18, 0.22, 0.05), (0.0, 0.02, 2.375), powder_coat)
    add_box(left_post, (0.06, 0.14, 0.34), (0.05, 0.11, 1.05), stainless)
    add_box(left_post, (0.035, 0.10, 0.75), (0.06, 0.08, 1.14), rubber)
    add_box(left_post, (0.05, 0.06, 0.10), (0.07, 0.09, 0.72), rubber)
    left_post.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 2.40)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.02, 1.20)),
    )

    right_post = model.part("right_post")
    add_box(right_post, (0.28, 0.24, 2.55), (0.0, 0.0, 1.275), powder_coat)
    add_box(right_post, (0.30, 0.26, 0.06), (0.0, 0.02, 2.58), powder_coat)
    add_box(right_post, (0.52, 0.12, 0.12), (-0.36, 0.14, 1.86), galvanized)
    add_box(right_post, (0.16, 0.18, 0.14), (-0.60, 0.14, 1.86), galvanized)
    add_cylinder(
        right_post,
        0.045,
        0.040,
        (-0.60, 0.105, 1.86),
        rubber,
        rpy=(0.0, math.pi * 0.5, 0.0),
    )
    add_cylinder(
        right_post,
        0.045,
        0.040,
        (-0.60, 0.175, 1.86),
        rubber,
        rpy=(0.0, math.pi * 0.5, 0.0),
    )
    add_box(right_post, (0.46, 0.34, 0.88), (0.06, -0.28, 0.44), powder_coat)
    add_box(right_post, (0.30, 0.03, 0.60), (0.06, -0.44, 0.46), stainless)
    add_box(right_post, (0.34, 0.22, 0.24), (-0.02, -0.17, 0.12), galvanized)
    add_box(right_post, (0.05, 0.06, 0.10), (-0.12, 0.10, 0.72), rubber)
    add_cylinder(right_post, 0.07, 0.12, (0.0, 0.02, 2.67), amber)
    add_cylinder(
        right_post,
        0.05,
        0.02,
        (0.0, 0.02, 2.74),
        stainless,
    )
    right_post.inertial = Inertial.from_geometry(
        Box((0.70, 0.50, 2.75)),
        mass=152.0,
        origin=Origin(xyz=(0.0, -0.08, 1.375)),
    )

    access_pedestal = model.part("access_pedestal")
    add_box(access_pedestal, (0.18, 0.20, 0.04), (0.0, 0.0, 0.02), galvanized)
    add_box(access_pedestal, (0.12, 0.14, 1.20), (0.0, 0.0, 0.64), powder_coat)
    add_box(access_pedestal, (0.07, 0.04, 0.12), (0.06, -0.09, 1.02), stainless)
    add_box(access_pedestal, (0.08, 0.05, 0.18), (0.06, -0.095, 0.80), stainless)
    add_box(access_pedestal, (0.14, 0.10, 0.04), (0.01, -0.11, 1.14), powder_coat)
    add_cylinder(
        access_pedestal,
        0.012,
        0.012,
        (0.072, -0.111, 1.00),
        amber,
        rpy=(math.pi * 0.5, 0.0, 0.0),
    )
    access_pedestal.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 1.24)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    gate_leaf = model.part("gate_leaf")
    add_box(gate_leaf, (0.22, 0.14, 0.10), (0.10, 0.0, 0.056), galvanized)
    add_box(gate_leaf, (0.22, 0.14, 0.10), (-0.65, 0.0, 0.056), galvanized)
    add_box(gate_leaf, (3.30, 0.08, 0.08), (-0.50, 0.0, 0.14), powder_coat)
    add_box(gate_leaf, (3.30, 0.08, 0.08), (-0.50, 0.0, 1.86), powder_coat)
    add_box(gate_leaf, (0.08, 0.08, 1.72), (-2.11, 0.0, 1.00), powder_coat)
    add_box(gate_leaf, (0.08, 0.08, 1.72), (1.11, 0.0, 1.00), powder_coat)
    add_box(gate_leaf, (0.08, 0.08, 1.72), (0.00, 0.0, 1.00), powder_coat)
    add_box(gate_leaf, (0.06, 0.06, 1.72), (-0.95, 0.0, 1.00), powder_coat)
    add_box(gate_leaf, (2.18, 0.05, 0.05), (-1.06, 0.0, 1.02), powder_coat)
    add_box(gate_leaf, (2.18, 0.04, 0.04), (-1.06, 0.0, 0.42), powder_coat)
    for x in (-1.90, -1.69, -1.48, -1.27, -1.06, -0.85, -0.64, -0.43, -0.22):
        add_box(gate_leaf, (0.025, 0.025, 1.64), (x, 0.0, 1.00), galvanized)
    add_brace(gate_leaf, (0.02, 0.0, 0.18), (1.07, 0.0, 1.82), 0.04, 0.04, powder_coat)
    add_brace(gate_leaf, (0.02, 0.0, 1.82), (1.07, 0.0, 0.18), 0.04, 0.04, powder_coat)
    add_brace(gate_leaf, (-0.65, 0.0, 0.18), (0.00, 0.0, 1.82), 0.04, 0.04, powder_coat)
    add_box(gate_leaf, (0.10, 0.10, 0.28), (-2.06, 0.0, 1.05), stainless)
    add_box(gate_leaf, (0.020, 0.12, 1.60), (-2.16, 0.0, 1.00), rubber)
    add_box(gate_leaf, (0.14, 0.10, 0.08), (0.08, 0.0, 1.80), galvanized)
    gate_leaf.inertial = Inertial.from_geometry(
        Box((3.32, 0.16, 1.90)),
        mass=184.0,
        origin=Origin(xyz=(-0.50, 0.0, 0.95)),
    )

    model.articulation(
        "curb_to_track",
        ArticulationType.FIXED,
        parent="curb_base",
        child="ground_track",
        origin=Origin(xyz=(TRACK_CENTER_X, TRACK_Y, MOUNT_Z)),
    )
    model.articulation(
        "curb_to_left_post",
        ArticulationType.FIXED,
        parent="curb_base",
        child="left_post",
        origin=Origin(xyz=(-1.05, -0.16, MOUNT_Z)),
    )
    model.articulation(
        "curb_to_right_post",
        ArticulationType.FIXED,
        parent="curb_base",
        child="right_post",
        origin=Origin(xyz=(1.15, -0.16, MOUNT_Z)),
    )
    model.articulation(
        "curb_to_access_pedestal",
        ArticulationType.FIXED,
        parent="curb_base",
        child="access_pedestal",
        origin=Origin(xyz=(1.62, -0.27, MOUNT_Z)),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent="ground_track",
        child="gate_leaf",
        origin=Origin(xyz=(GATE_JOINT_X, 0.0, TRACK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.45,
            lower=0.0,
            upper=GATE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    def axis_value(aabb, which: str, axis: str) -> float:
        idx = {"x": 0, "y": 1, "z": 2}[axis]
        for attr in (f"{which}_{axis}", f"{axis}_{which}", f"{which}{axis}"):
            if hasattr(aabb, attr):
                return float(getattr(aabb, attr))
        endpoint = getattr(aabb, which, None)
        if endpoint is not None:
            if hasattr(endpoint, axis):
                return float(getattr(endpoint, axis))
            try:
                return float(endpoint[idx])
            except (TypeError, IndexError):
                pass
        try:
            return float(aabb[0 if which == "min" else 1][idx])
        except (TypeError, IndexError, KeyError):
            raise AssertionError(f"Unsupported AABB format for axis lookup: {aabb!r}")

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_gap("ground_track", "curb_base", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("left_post", "curb_base", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("right_post", "curb_base", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("access_pedestal", "curb_base", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("gate_leaf", "ground_track", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_overlap("gate_leaf", "ground_track", axes="xy", min_overlap=0.08)
    ctx.expect_joint_motion_axis(
        "gate_slide",
        "gate_leaf",
        world_axis="x",
        direction="positive",
        min_delta=1.50,
    )

    left_post_aabb = ctx.part_world_aabb("left_post", use="collision")
    gate_closed_aabb = ctx.part_world_aabb("gate_leaf", use="collision")
    pedestal_aabb = ctx.part_world_aabb("access_pedestal", use="collision")

    closed_leading_clearance = axis_value(gate_closed_aabb, "min", "x") - axis_value(
        left_post_aabb, "max", "x"
    )
    require(
        -0.03 <= closed_leading_clearance <= 0.08,
        f"Closed gate should sit tightly at the receiver post; got {closed_leading_clearance:.3f} m.",
    )

    front_service_clearance = axis_value(gate_closed_aabb, "min", "y") - axis_value(
        pedestal_aabb, "max", "y"
    )
    require(
        front_service_clearance >= 0.15,
        f"Access pedestal should stay clear of the sliding envelope; got {front_service_clearance:.3f} m.",
    )

    with ctx.pose(gate_slide=GATE_TRAVEL):
        ctx.expect_aabb_gap("gate_leaf", "ground_track", axis="z", max_gap=0.010, max_penetration=0.0)
        ctx.expect_aabb_overlap("gate_leaf", "ground_track", axes="xy", min_overlap=0.08)

        gate_open_aabb = ctx.part_world_aabb("gate_leaf", use="collision")
        track_aabb = ctx.part_world_aabb("ground_track", use="collision")

        open_leading_clearance = axis_value(gate_open_aabb, "min", "x") - axis_value(
            left_post_aabb, "max", "x"
        )
        require(
            open_leading_clearance >= 1.80,
            f"Open gate should clear the driveway opening; got {open_leading_clearance:.3f} m.",
        )

        trailing_support_margin = axis_value(track_aabb, "max", "x") - axis_value(
            gate_open_aabb, "max", "x"
        )
        require(
            trailing_support_margin >= 0.04,
            f"Gate tail should remain on the support track at full open; got {trailing_support_margin:.3f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
