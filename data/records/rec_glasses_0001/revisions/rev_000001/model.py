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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

FRAME_THICKNESS = 0.0052
LENS_OFFSET_X = 0.034
LENS_CENTER_Z = 0.0012
LEFT_HINGE_ORIGIN = (-0.0685, -0.0028, 0.0105)
RIGHT_HINGE_ORIGIN = (0.0685, -0.0042, 0.0105)


def _make_material(name, rgba):
    constructor_variants = (
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name=name, diffuse=rgba),
        lambda: Material(name=name, diffuse_color=rgba),
        lambda: Material(name=name, albedo=rgba),
        lambda: Material(name, rgba),
    )
    for variant in constructor_variants:
        try:
            return variant()
        except TypeError:
            continue

    material = Material(name=name)
    for attr in ("rgba", "diffuse", "diffuse_color", "albedo", "color"):
        if hasattr(material, attr):
            setattr(material, attr, rgba)
            break
    return material


def _translate_profile(profile, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _build_front_frame_mesh():
    outer_points = [
        (-0.067, -0.018),
        (-0.071, 0.003),
        (-0.066, 0.022),
        (-0.053, 0.031),
        (-0.030, 0.030),
        (-0.014, 0.025),
        (-0.006, 0.019),
        (0.006, 0.019),
        (0.014, 0.025),
        (0.030, 0.030),
        (0.053, 0.031),
        (0.066, 0.022),
        (0.071, 0.003),
        (0.067, -0.018),
        (0.050, -0.026),
        (0.029, -0.029),
        (0.010, -0.023),
        (0.000, -0.017),
        (-0.010, -0.023),
        (-0.029, -0.029),
        (-0.050, -0.026),
    ]
    outer_profile = sample_catmull_rom_spline_2d(
        outer_points,
        samples_per_segment=10,
        closed=True,
    )
    lens_cutout = rounded_rect_profile(0.048, 0.032, radius=0.0085, corner_segments=8)
    frame_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [
            _translate_profile(lens_cutout, -LENS_OFFSET_X, LENS_CENTER_Z),
            _translate_profile(lens_cutout, LENS_OFFSET_X, LENS_CENTER_Z),
        ],
        FRAME_THICKNESS,
        cap=True,
        center=True,
        closed=True,
    )
    frame_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(frame_geom, ASSETS.mesh_path("glasses_front_frame.obj"))


def _build_lens_mesh():
    lens_profile = rounded_rect_profile(0.0488, 0.0328, radius=0.0086, corner_segments=8)
    lens_geom = ExtrudeGeometry(
        lens_profile,
        0.0018,
        cap=True,
        center=True,
        closed=True,
    )
    lens_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(lens_geom, ASSETS.mesh_path("glasses_lens.obj"))


def _build_temple_body_mesh():
    temple_profile = rounded_rect_profile(0.0026, 0.0088, radius=0.00065, corner_segments=6)
    temple_path = [
        (0.0, -0.006, 0.0000),
        (0.0, -0.025, 0.0003),
        (0.0, -0.062, -0.0012),
        (0.0, -0.101, -0.0070),
        (0.0, -0.131, -0.0190),
        (0.0, -0.145, -0.0350),
    ]
    temple_geom = sweep_profile_along_spline(
        temple_path,
        profile=temple_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    temple_geom.merge(BoxGeometry((0.0056, 0.0100, 0.0090)).translate(0.0, -0.0050, 0.0))
    return mesh_from_geometry(temple_geom, ASSETS.mesh_path("glasses_temple_body.obj"))


def _build_temple_tip_mesh():
    tip_profile = rounded_rect_profile(0.0021, 0.0070, radius=0.00055, corner_segments=6)
    tip_path = [
        (0.0, -0.093, -0.0040),
        (0.0, -0.114, -0.0120),
        (0.0, -0.132, -0.0240),
        (0.0, -0.145, -0.0380),
    ]
    tip_geom = sweep_profile_along_spline(
        tip_path,
        profile=tip_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    return mesh_from_geometry(tip_geom, ASSETS.mesh_path("glasses_temple_tip.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="eyeglasses", assets=ASSETS)

    black_acetate = _make_material("black_acetate", (0.10, 0.10, 0.11, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.74, 0.76, 0.79, 1.0))
    smoked_lens = _make_material("smoked_lens", (0.43, 0.46, 0.50, 0.28))
    clear_pad = _make_material("clear_pad", (0.92, 0.94, 0.97, 0.55))
    rubber_tip = _make_material("rubber_tip", (0.06, 0.06, 0.07, 1.0))

    frame_mesh = _build_front_frame_mesh()
    lens_mesh = _build_lens_mesh()
    temple_body_mesh = _build_temple_body_mesh()
    temple_tip_mesh = _build_temple_tip_mesh()

    frame = model.part("frame_front")
    frame.visual(frame_mesh, material=black_acetate, name="front_frame")
    frame.visual(
        lens_mesh,
        origin=Origin(xyz=(-LENS_OFFSET_X, 0.0, LENS_CENTER_Z)),
        material=smoked_lens,
        name="left_lens",
    )
    frame.visual(
        lens_mesh,
        origin=Origin(xyz=(LENS_OFFSET_X, 0.0, LENS_CENTER_Z)),
        material=smoked_lens,
        name="right_lens",
    )
    frame.visual(
        Box((0.011, 0.0034, 0.0085)),
        origin=Origin(xyz=(0.0, -0.0014, -0.0102)),
        material=black_acetate,
        name="bridge_saddle",
    )
    frame.visual(
        Box((0.0018, 0.0052, 0.0120)),
        origin=Origin(xyz=(-0.0083, -0.0027, -0.0080)),
        material=brushed_steel,
        name="left_pad_arm",
    )
    frame.visual(
        Box((0.0018, 0.0052, 0.0120)),
        origin=Origin(xyz=(0.0083, -0.0027, -0.0080)),
        material=brushed_steel,
        name="right_pad_arm",
    )
    frame.visual(
        Sphere(radius=0.0034),
        origin=Origin(xyz=(-0.0106, -0.0043, -0.0120)),
        material=clear_pad,
        name="left_nose_pad",
    )
    frame.visual(
        Sphere(radius=0.0034),
        origin=Origin(xyz=(0.0106, -0.0043, -0.0120)),
        material=clear_pad,
        name="right_nose_pad",
    )
    frame.visual(
        Box((0.0040, 0.0060, 0.0110)),
        origin=Origin(xyz=LEFT_HINGE_ORIGIN),
        material=brushed_steel,
        name="left_hinge_plate",
    )
    frame.visual(
        Box((0.0040, 0.0070, 0.0110)),
        origin=Origin(xyz=RIGHT_HINGE_ORIGIN),
        material=brushed_steel,
        name="right_hinge_plate",
    )
    frame.visual(
        Cylinder(radius=0.0018, length=0.0052),
        origin=Origin(xyz=LEFT_HINGE_ORIGIN),
        material=brushed_steel,
        name="left_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.0018, length=0.0052),
        origin=Origin(xyz=RIGHT_HINGE_ORIGIN),
        material=brushed_steel,
        name="right_hinge_barrel",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.145, 0.018, 0.062)),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    left_temple = model.part("left_temple")
    left_temple.visual(temple_body_mesh, material=black_acetate, name="left_temple_body")
    left_temple.visual(temple_tip_mesh, material=rubber_tip, name="left_temple_tip")
    left_temple.visual(
        Cylinder(radius=0.0016, length=0.0042),
        origin=Origin(),
        material=brushed_steel,
        name="left_temple_knuckle",
    )
    left_temple.visual(
        Box((0.0010, 0.028, 0.0012)),
        origin=Origin(xyz=(0.0, -0.028, 0.0026)),
        material=brushed_steel,
        name="left_core_inlay",
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.010, 0.150, 0.042)),
        mass=0.007,
        origin=Origin(xyz=(0.0, -0.074, -0.013)),
    )

    right_temple = model.part("right_temple")
    right_temple.visual(temple_body_mesh, material=black_acetate, name="right_temple_body")
    right_temple.visual(temple_tip_mesh, material=rubber_tip, name="right_temple_tip")
    right_temple.visual(
        Cylinder(radius=0.0016, length=0.0042),
        origin=Origin(),
        material=brushed_steel,
        name="right_temple_knuckle",
    )
    right_temple.visual(
        Box((0.0010, 0.028, 0.0012)),
        origin=Origin(xyz=(0.0, -0.028, 0.0026)),
        material=brushed_steel,
        name="right_core_inlay",
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.010, 0.150, 0.042)),
        mass=0.007,
        origin=Origin(xyz=(0.0, -0.074, -0.013)),
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent="frame_front",
        child="left_temple",
        origin=Origin(xyz=LEFT_HINGE_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=5.0,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent="frame_front",
        child="right_temple",
        origin=Origin(xyz=RIGHT_HINGE_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=5.0,
            lower=0.0,
            upper=1.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _bounds(aabb):
        attr_layouts = [
            ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z"),
            ("xmin", "ymin", "zmin", "xmax", "ymax", "zmax"),
        ]
        for attrs in attr_layouts:
            if all(hasattr(aabb, attr) for attr in attrs):
                return tuple(float(getattr(aabb, attr)) for attr in attrs)
        if hasattr(aabb, "min") and hasattr(aabb, "max"):
            mins = tuple(float(v) for v in aabb.min)
            maxs = tuple(float(v) for v in aabb.max)
            return (*mins, *maxs)
        if hasattr(aabb, "mins") and hasattr(aabb, "maxs"):
            mins = tuple(float(v) for v in aabb.mins)
            maxs = tuple(float(v) for v in aabb.maxs)
            return (*mins, *maxs)
        if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
            mins = tuple(float(v) for v in aabb[0])
            maxs = tuple(float(v) for v in aabb[1])
            return (*mins, *maxs)
        raise AssertionError(f"Unsupported AABB representation: {type(aabb)!r}")

    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "frame_front",
        "left_temple",
        reason="The left hinge knuckle intentionally nests into the frame-side barrel at the pivot.",
    )
    ctx.allow_overlap(
        "frame_front",
        "right_temple",
        reason="The right hinge knuckle intentionally nests into the frame-side barrel at the pivot.",
    )
    ctx.allow_overlap(
        "left_temple",
        "right_temple",
        reason="Folded temple arms stack tightly and convex collision hulls are conservative around the ear hooks.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "left_hinge", "left_temple", world_axis="x", direction="positive", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "left_hinge", "left_temple", world_axis="y", direction="positive", min_delta=0.05
    )
    ctx.expect_joint_motion_axis(
        "right_hinge", "right_temple", world_axis="x", direction="negative", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "right_hinge", "right_temple", world_axis="y", direction="positive", min_delta=0.05
    )

    frame_bounds = _bounds(ctx.part_world_aabb("frame_front", use="collision"))
    left_open_bounds = _bounds(ctx.part_world_aabb("left_temple", use="collision"))
    right_open_bounds = _bounds(ctx.part_world_aabb("right_temple", use="collision"))

    frame_width = frame_bounds[3] - frame_bounds[0]
    frame_height = frame_bounds[5] - frame_bounds[2]
    assert 0.132 <= frame_width <= 0.148, (
        f"frame width {frame_width:.4f} m should read as adult eyewear"
    )
    assert 0.050 <= frame_height <= 0.070, (
        f"frame height {frame_height:.4f} m should match a wearable eyeglass front"
    )
    assert left_open_bounds[1] < frame_bounds[1] - 0.050, (
        "left temple should extend well behind the frame in the open pose"
    )
    assert right_open_bounds[1] < frame_bounds[1] - 0.050, (
        "right temple should extend well behind the frame in the open pose"
    )
    assert left_open_bounds[3] < 0.0, (
        "left temple should remain on the wearer's left side when open"
    )
    assert right_open_bounds[0] > 0.0, (
        "right temple should remain on the wearer's right side when open"
    )
    assert -0.074 <= left_open_bounds[3] <= -0.063, (
        "left temple hinge end should stay tucked at the left frame edge"
    )
    assert 0.063 <= right_open_bounds[0] <= 0.074, (
        "right temple hinge end should stay tucked at the right frame edge"
    )
    assert -0.008 <= left_open_bounds[4] <= 0.008, (
        "left temple should attach near the frame's rear edge"
    )
    assert -0.008 <= right_open_bounds[4] <= 0.008, (
        "right temple should attach near the frame's rear edge"
    )

    with ctx.pose(left_hinge=1.22):
        left_folded_bounds = _bounds(ctx.part_world_aabb("left_temple", use="collision"))
        assert left_folded_bounds[3] > -0.010, "left temple should fold inward toward the bridge"
        assert left_folded_bounds[1] > left_open_bounds[1] + 0.070, (
            "left temple should move substantially forward when folded"
        )

    with ctx.pose(right_hinge=1.22):
        right_folded_bounds = _bounds(ctx.part_world_aabb("right_temple", use="collision"))
        assert right_folded_bounds[0] < 0.010, "right temple should fold inward toward the bridge"
        assert right_folded_bounds[1] > right_open_bounds[1] + 0.070, (
            "right temple should move substantially forward when folded"
        )

    with ctx.pose(left_hinge=1.22, right_hinge=1.22):
        left_both_folded = _bounds(ctx.part_world_aabb("left_temple", use="collision"))
        right_both_folded = _bounds(ctx.part_world_aabb("right_temple", use="collision"))
        folded_span = max(frame_bounds[3], left_both_folded[3], right_both_folded[3]) - min(
            frame_bounds[0],
            left_both_folded[0],
            right_both_folded[0],
        )
        assert folded_span < 0.155, (
            "folded glasses should stay compact enough to read as a consumer eyewear product"
        )
        assert left_both_folded[3] > right_both_folded[0] - 0.012, (
            "folded temples should nest into a compact stacked package"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
