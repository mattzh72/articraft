from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _rounded_ring_mesh(
    filename: str,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    height: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_size[0],
            outer_size[1],
            outer_radius,
            corner_segments=10,
        ),
        [
            rounded_rect_profile(
                inner_size[0],
                inner_size[1],
                inner_radius,
                corner_segments=10,
            )
        ],
        height=height,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _pull_handle_mesh(filename: str):
    geom = tube_from_spline_points(
        [
            (-0.060, 0.0, 0.000),
            (-0.045, 0.0, 0.024),
            (0.045, 0.0, 0.024),
            (0.060, 0.0, 0.000),
        ],
        radius=0.006,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_deck_hatch", assets=ASSETS)

    powder_white = _make_material("powder_white", (0.94, 0.95, 0.95, 1.0))
    brushed_aluminum = _make_material("brushed_aluminum", (0.74, 0.75, 0.77, 1.0))
    stainless_steel = _make_material("stainless_steel", (0.70, 0.72, 0.75, 1.0))
    smoked_acrylic = _make_material("smoked_acrylic", (0.48, 0.60, 0.70, 0.35))
    black_rubber = _make_material("black_rubber", (0.08, 0.08, 0.09, 1.0))
    black_polymer = _make_material("black_polymer", (0.12, 0.12, 0.13, 1.0))
    model.materials.extend(
        [
            powder_white,
            brushed_aluminum,
            stainless_steel,
            smoked_acrylic,
            black_rubber,
            black_polymer,
        ]
    )

    flange_height = 0.012
    curb_height = 0.048
    frame_top = flange_height + curb_height
    hinge_y = -0.275

    flange_outer = (0.82, 0.62)
    flange_inner = (0.70, 0.50)
    curb_outer = (0.76, 0.56)
    curb_inner = (0.69, 0.49)
    gasket_outer = (0.71, 0.51)
    gasket_inner = (0.67, 0.47)
    sash_outer = (0.75, 0.55)
    sash_inner = (0.66, 0.46)
    sash_height = 0.028
    sash_bottom = 0.003
    sash_center_z = sash_bottom + 0.5 * sash_height
    pull_handle_mesh = _pull_handle_mesh("pull_handle.obj")

    frame = model.part("frame")
    for x in (-0.380, 0.380):
        frame.visual(
            Box((0.060, flange_outer[1], flange_height)),
            origin=Origin(xyz=(x, 0.0, flange_height * 0.5)),
            material=powder_white,
            name=f"flange_side_{x:+.3f}",
        )
    for y in (-0.280, 0.280):
        frame.visual(
            Box((flange_inner[0], 0.060, flange_height)),
            origin=Origin(xyz=(0.0, y, flange_height * 0.5)),
            material=powder_white,
            name=f"flange_end_{y:+.3f}",
        )
    for x in (-0.3625, 0.3625):
        frame.visual(
            Box((0.035, curb_outer[1], curb_height)),
            origin=Origin(xyz=(x, 0.0, flange_height + curb_height * 0.5)),
            material=powder_white,
            name=f"curb_side_{x:+.4f}",
        )
    for y in (-0.2625, 0.2625):
        frame.visual(
            Box((curb_inner[0], 0.035, curb_height)),
            origin=Origin(xyz=(0.0, y, flange_height + curb_height * 0.5)),
            material=powder_white,
            name=f"curb_end_{y:+.4f}",
        )
    for x in (-0.345, 0.345):
        frame.visual(
            Box((0.020, gasket_outer[1], 0.003)),
            origin=Origin(xyz=(x, 0.0, frame_top - 0.0015)),
            material=black_rubber,
            name=f"gasket_side_{x:+.3f}",
        )
    for y in (-0.245, 0.245):
        frame.visual(
            Box((gasket_inner[0], 0.020, 0.003)),
            origin=Origin(xyz=(0.0, y, frame_top - 0.0015)),
            material=black_rubber,
            name=f"gasket_end_{y:+.3f}",
        )
    frame.visual(
        Box((0.70, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, -0.288, frame_top - 0.006)),
        material=stainless_steel,
        name="hinge_backing_rail",
    )
    frame.visual(
        Box((0.082, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.286, frame_top - 0.006)),
        material=stainless_steel,
        name="latch_strike",
    )
    for x in (-0.330, -0.165, 0.0, 0.165, 0.330):
        for y in (-0.250, 0.250):
            frame.visual(
                Cylinder(radius=0.007, length=0.003),
                origin=Origin(xyz=(x, y, flange_height + 0.0015)),
                material=stainless_steel,
                name=f"fastener_{x:+.3f}_{y:+.3f}",
            )
    for x in (-0.336, 0.336):
        frame.visual(
            Box((0.022, 0.070, 0.028)),
            origin=Origin(xyz=(0.352 if x > 0.0 else -0.352, 0.145, 0.032)),
            material=stainless_steel,
            name=f"stay_bracket_{x:+.3f}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.62, 0.060)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    lid = model.part("lid")
    for x in (-0.3525, 0.3525):
        lid.visual(
            Box((0.045, sash_inner[1], sash_height)),
            origin=Origin(xyz=(x, 0.275, sash_center_z)),
            material=brushed_aluminum,
            name=f"sash_side_{x:+.4f}",
        )
    for y in (0.0225, 0.5275):
        lid.visual(
            Box((sash_outer[0], 0.045, sash_height)),
            origin=Origin(xyz=(0.0, y, sash_center_z)),
            material=brushed_aluminum,
            name=f"sash_end_{y:+.4f}",
        )
    lid.visual(
        Box((0.664, 0.464, 0.008)),
        origin=Origin(xyz=(0.0, 0.275, 0.016)),
        material=smoked_acrylic,
        name="glazing_panel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.680),
        origin=Origin(xyz=(0.0, 0.010, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless_steel,
        name="hinge_knuckle",
    )
    for x in (-0.220, 0.220):
        lid.visual(
            Box((0.120, 0.018, 0.005)),
            origin=Origin(xyz=(x, 0.012, 0.0125)),
            material=stainless_steel,
            name=f"hinge_strap_{x:+.3f}",
        )
    for x in (-0.337, 0.337):
        lid.visual(
            Box((0.016, 0.060, 0.016)),
            origin=Origin(xyz=(x, 0.220, 0.018)),
            material=stainless_steel,
            name=f"stay_tab_{x:+.3f}",
        )
    lid.visual(
        Box((0.108, 0.036, 0.028)),
        origin=Origin(xyz=(0.0, 0.518, 0.031)),
        material=black_polymer,
        name="latch_housing",
    )
    lid.visual(
        pull_handle_mesh,
        origin=Origin(xyz=(0.0, 0.505, 0.045)),
        material=black_polymer,
        name="pull_handle",
    )
    lid.visual(
        Box((0.040, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.545, 0.018)),
        material=stainless_steel,
        name="latch_bolt",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.75, 0.55, 0.036)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.275, 0.018)),
    )

    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="lid",
        origin=Origin(xyz=(0.0, hinge_y, frame_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    articulation = object_model.get_articulation("frame_to_lid")
    assert articulation.axis == (1.0, 0.0, 0.0), "Hatch should hinge around the lateral axis."
    assert articulation.motion_limits is not None, "Hatch needs explicit opening limits."
    assert articulation.motion_limits.lower <= 0.0 + 1e-9
    assert articulation.motion_limits.upper >= 1.30

    ctx.expect_aabb_overlap("lid", "frame", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("lid", "frame", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "frame_to_lid",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(frame_to_lid=0.70):
        ctx.expect_aabb_overlap("lid", "frame", axes="xy", min_overlap=0.12)

    with ctx.pose(frame_to_lid=1.30):
        ctx.expect_aabb_overlap("lid", "frame", axes="xy", min_overlap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
