from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            return Material(name, rgba)


def _add_box(part, name, size, xyz, material=None, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size=size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material=None, rpy=(0.0, pi / 2.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate", assets=ASSETS)

    painted_steel = _material("painted_steel", (0.16, 0.18, 0.20, 1.0))
    satin_black = _material("satin_black", (0.07, 0.07, 0.08, 1.0))
    dark_steel = _material("dark_steel", (0.34, 0.35, 0.37, 1.0))
    rubber = _material("rubber", (0.03, 0.03, 0.03, 1.0))
    if hasattr(model, "materials"):
        model.materials.extend([painted_steel, satin_black, dark_steel, rubber])

    bed_floor = model.part("bed_floor")
    _add_box(
        bed_floor,
        "load_floor",
        (1.62, 0.84, 0.045),
        (0.0, 0.42, 0.5375),
        painted_steel,
    )
    _add_box(
        bed_floor,
        "rear_sill",
        (1.62, 0.09, 0.055),
        (0.0, 0.025, 0.5325),
        painted_steel,
    )
    _add_box(
        bed_floor,
        "lower_reinforcement",
        (1.62, 0.06, 0.10),
        (0.0, -0.035, 0.485),
        dark_steel,
    )
    _add_box(
        bed_floor,
        "hinge_carrier",
        (0.06, 0.024, 0.02),
        (0.0, 0.0, 0.56),
        dark_steel,
    )
    for side in (-1.0, 1.0):
        x = side * 0.69
        _add_box(
            bed_floor,
            f"hinge_pedestal_{'l' if side < 0 else 'r'}",
            (0.12, 0.08, 0.08),
            (x, 0.01, 0.56),
            painted_steel,
        )
        _add_cylinder(
            bed_floor,
            f"hinge_socket_{'l' if side < 0 else 'r'}",
            0.024,
            0.065,
            (x, 0.0, 0.56),
            dark_steel,
        )
    bed_floor.inertial = Inertial.from_geometry(
        Box(size=(1.62, 0.84, 0.12)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.39, 0.54)),
    )

    body_frame = model.part("body_frame")
    body_frame_origin = (0.705, 0.02, 0.56)

    def body_xyz(x: float, y: float, z: float) -> tuple[float, float, float]:
        return (x - body_frame_origin[0], y - body_frame_origin[1], z - body_frame_origin[2])

    for side in (-1.0, 1.0):
        tag = "l" if side < 0 else "r"
        outer_x = side * 0.775
        jamb_x = side * 0.705
        _add_box(
            body_frame,
            f"bedside_outer_{tag}",
            (0.11, 0.88, 0.60),
            body_xyz(outer_x, 0.39, 0.85),
            painted_steel,
        )
        _add_box(
            body_frame,
            f"bedside_cap_{tag}",
            (0.11, 0.88, 0.018),
            body_xyz(outer_x, 0.39, 1.159),
            satin_black,
        )
        _add_box(
            body_frame,
            f"rear_jamb_{tag}",
            (0.05, 0.16, 0.28),
            body_xyz(jamb_x, 0.02, 0.69),
            painted_steel,
        )
        _add_box(
            body_frame,
            f"rear_buttress_{tag}",
            (0.14, 0.10, 0.24),
            body_xyz(side * 0.76, 0.02, 0.97),
            painted_steel,
        )
        _add_box(
            body_frame,
            f"striker_pad_{tag}",
            (0.04, 0.04, 0.08),
            body_xyz(side * 0.67, -0.012, 0.93),
            rubber,
        )
    _add_box(
        body_frame,
        "front_bulkhead",
        (1.46, 0.07, 0.50),
        body_xyz(0.0, 0.815, 0.80),
        painted_steel,
    )
    _add_box(
        body_frame,
        "front_bulkhead_cap",
        (1.46, 0.07, 0.018),
        body_xyz(0.0, 0.815, 1.159),
        satin_black,
    )
    body_frame.inertial = Inertial.from_geometry(
        Box(size=(1.66, 0.90, 0.62)),
        mass=46.0,
        origin=Origin(xyz=body_xyz(0.0, 0.41, 0.86)),
    )

    tailgate = model.part("tailgate")
    _add_box(
        tailgate,
        "outer_frame_top",
        (1.50, 0.032, 0.12),
        (0.0, -0.016, 0.50),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_frame_bottom",
        (1.50, 0.032, 0.12),
        (0.0, -0.016, 0.06),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_frame_left",
        (0.11, 0.032, 0.56),
        (-0.695, -0.016, 0.28),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_frame_right",
        (0.11, 0.032, 0.56),
        (0.695, -0.016, 0.28),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_center_recess",
        (1.28, 0.016, 0.32),
        (0.0, -0.008, 0.28),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_belt_panel",
        (1.08, 0.010, 0.11),
        (0.0, -0.021, 0.35),
        painted_steel,
    )
    _add_box(
        tailgate,
        "outer_handle_bezel",
        (0.22, 0.014, 0.055),
        (0.0, -0.022, 0.50),
        satin_black,
    )
    _add_box(
        tailgate,
        "top_cap",
        (1.52, 0.048, 0.022),
        (0.0, 0.008, 0.549),
        satin_black,
    )
    _add_box(
        tailgate,
        "side_return_left",
        (0.026, 0.068, 0.56),
        (-0.737, 0.006, 0.28),
        painted_steel,
    )
    _add_box(
        tailgate,
        "side_return_right",
        (0.026, 0.068, 0.56),
        (0.737, 0.006, 0.28),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_top_rib",
        (1.40, 0.050, 0.075),
        (0.0, 0.014, 0.495),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_bottom_beam",
        (1.46, 0.055, 0.10),
        (0.0, 0.014, 0.05),
        dark_steel,
    )
    _add_box(
        tailgate,
        "inner_side_rib_left",
        (0.08, 0.060, 0.42),
        (-0.67, 0.014, 0.27),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_side_rib_right",
        (0.08, 0.060, 0.42),
        (0.67, 0.014, 0.27),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_center_panel",
        (1.18, 0.030, 0.30),
        (0.0, 0.022, 0.27),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_stiffener_left",
        (0.09, 0.028, 0.18),
        (-0.32, 0.024, 0.21),
        painted_steel,
    )
    _add_box(
        tailgate,
        "inner_stiffener_right",
        (0.09, 0.028, 0.18),
        (0.32, 0.024, 0.21),
        painted_steel,
    )
    _add_box(
        tailgate,
        "latch_housing_left",
        (0.08, 0.048, 0.08),
        (-0.62, 0.028, 0.42),
        dark_steel,
    )
    _add_box(
        tailgate,
        "latch_housing_right",
        (0.08, 0.048, 0.08),
        (0.62, 0.028, 0.42),
        dark_steel,
    )
    _add_box(
        tailgate,
        "inner_access_trim",
        (0.32, 0.010, 0.08),
        (0.0, 0.039, 0.50),
        satin_black,
    )
    _add_box(
        tailgate,
        "axis_pad",
        (0.05, 0.02, 0.02),
        (0.0, 0.0, 0.01),
        dark_steel,
    )
    for side in (-1.0, 1.0):
        tag = "l" if side < 0 else "r"
        _add_cylinder(
            tailgate,
            f"hinge_knuckle_{tag}",
            0.022,
            0.070,
            (side * 0.69, 0.0, 0.032),
            dark_steel,
        )
    tailgate.inertial = Inertial.from_geometry(
        Box(size=(1.52, 0.06, 0.56)),
        mass=27.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    model.articulation(
        "bed_floor_to_body_frame",
        ArticulationType.FIXED,
        parent="bed_floor",
        child="body_frame",
        origin=Origin(xyz=body_frame_origin),
    )
    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent="bed_floor",
        child="tailgate",
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.4,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "tailgate",
        "body_frame",
        reason="Closed tailgate side returns nest tightly inside the rear bedside jambs.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("tailgate", "body_frame", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("body_frame", "bed_floor", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("tailgate", "bed_floor", axes="xy", min_overlap=0.05)
    ctx.expect_joint_motion_axis(
        "tailgate_hinge",
        "tailgate",
        world_axis="z",
        direction="negative",
        min_delta=0.12,
    )

    with ctx.pose(tailgate_hinge=pi / 4.0):
        ctx.expect_aabb_overlap("tailgate", "body_frame", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_overlap("tailgate", "bed_floor", axes="xy", min_overlap=0.05)

    with ctx.pose(tailgate_hinge=pi / 2.0):
        ctx.expect_aabb_overlap("tailgate", "body_frame", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_overlap("tailgate", "bed_floor", axes="xy", min_overlap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
