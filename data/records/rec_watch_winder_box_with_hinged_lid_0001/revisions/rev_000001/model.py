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
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            return Material(name, rgba)


def _mesh_for_geometry(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_solid_mesh(
    name: str,
    width: float,
    depth: float,
    radius: float,
    height: float,
    *,
    z0: float = 0.0,
):
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, radius),
        height=height,
        cap=True,
        closed=True,
    )
    if z0:
        geom.translate(0.0, 0.0, z0)
    return _mesh_for_geometry(name, geom)


def _rounded_ring_mesh(
    name: str,
    outer_width: float,
    outer_depth: float,
    outer_radius: float,
    inner_width: float,
    inner_depth: float,
    inner_radius: float,
    height: float,
    *,
    z0: float = 0.0,
    cap: bool = True,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_depth, outer_radius),
        [rounded_rect_profile(inner_width, inner_depth, inner_radius)],
        height=height,
        cap=cap,
        center=False,
        closed=True,
    )
    if z0:
        geom.translate(0.0, 0.0, z0)
    return _mesh_for_geometry(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luxury_watch_winder_box", assets=ASSETS)

    materials = {
        "walnut": _make_material("walnut_wood", (0.29, 0.19, 0.11, 1.0)),
        "lacquer": _make_material("dark_lacquer", (0.18, 0.11, 0.07, 1.0)),
        "velvet": _make_material("charcoal_velvet", (0.11, 0.11, 0.12, 1.0)),
        "steel": _make_material("brushed_steel", (0.79, 0.80, 0.82, 1.0)),
        "brass": _make_material("satin_brass", (0.73, 0.62, 0.39, 1.0)),
        "glass": _make_material("smoked_glass", (0.38, 0.44, 0.48, 0.28)),
        "leather": _make_material("black_leather", (0.10, 0.10, 0.11, 1.0)),
    }
    model.materials.extend(materials.values())

    outer_w = 0.310
    outer_d = 0.205
    base_h = 0.118
    wall = 0.018
    plinth_h = 0.010
    bottom_t = 0.014
    floor_z = plinth_h + bottom_t
    outer_r = 0.026
    inner_w = outer_w - 2.0 * wall
    inner_d = outer_d - 2.0 * wall
    inner_r = 0.012
    top_trim_t = 0.004

    lid_overhang = 0.004
    lid_w = outer_w + lid_overhang
    lid_d = outer_d + lid_overhang
    lid_r = 0.028
    lid_t = 0.032
    hinge_inset = 0.014
    lid_frame = 0.030
    lid_window_w = lid_w - 2.0 * lid_frame
    lid_window_d = lid_d - 2.0 * lid_frame
    lid_window_r = 0.012
    hinge_reveal = 0.001
    lid_y_offset = lid_d * 0.5 - hinge_inset

    body_shell = _rounded_ring_mesh(
        "body_shell.obj",
        outer_w,
        outer_d,
        outer_r,
        inner_w,
        inner_d,
        inner_r,
        base_h - plinth_h,
        z0=plinth_h,
        cap=False,
    )
    body_bottom = _rounded_solid_mesh(
        "body_bottom.obj",
        outer_w,
        outer_d,
        outer_r,
        bottom_t,
        z0=plinth_h,
    )
    body_plinth = _rounded_solid_mesh(
        "body_plinth.obj",
        outer_w * 0.93,
        outer_d * 0.79,
        0.022,
        plinth_h,
    )
    body_trim = _rounded_ring_mesh(
        "body_trim.obj",
        outer_w - 0.004,
        outer_d - 0.004,
        0.024,
        inner_w + 0.008,
        inner_d + 0.008,
        0.015,
        top_trim_t,
        z0=base_h - top_trim_t,
        cap=True,
    )
    body_lining = _rounded_ring_mesh(
        "body_lining.obj",
        inner_w + 0.002,
        inner_d + 0.002,
        inner_r + 0.001,
        inner_w - 0.022,
        inner_d - 0.022,
        0.006,
        base_h - floor_z - 0.010,
        z0=floor_z,
        cap=False,
    )

    lid_frame_mesh = _rounded_ring_mesh(
        "lid_frame.obj",
        lid_w,
        lid_d,
        lid_r,
        lid_window_w,
        lid_window_d,
        lid_window_r,
        lid_t,
        cap=True,
    )
    lid_bezel_mesh = _rounded_ring_mesh(
        "lid_bezel.obj",
        lid_window_w + 0.012,
        lid_window_d + 0.012,
        0.016,
        lid_window_w - 0.004,
        lid_window_d - 0.004,
        0.010,
        0.0025,
        z0=lid_t - 0.0065,
        cap=True,
    )
    lid_glass_mesh = _rounded_solid_mesh(
        "lid_glass.obj",
        lid_window_w + 0.010,
        lid_window_d + 0.010,
        0.016,
        0.004,
        z0=lid_t - 0.005,
    )

    body = model.part("body")
    body.visual(body_shell, material=materials["walnut"])
    body.visual(body_bottom, material=materials["lacquer"])
    body.visual(body_plinth, material=materials["lacquer"])
    body.visual(body_trim, material=materials["brass"])
    body.visual(body_lining, material=materials["velvet"])
    body.visual(
        Box((inner_w, inner_d, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, floor_z + 0.005)),
        material=materials["velvet"],
    )
    body.visual(
        Box((inner_w - 0.044, 0.096, 0.004)),
        origin=Origin(
            xyz=(0.0, 0.004, 0.031),
        ),
        material=materials["velvet"],
    )
    body.visual(
        Box((inner_w - 0.058, 0.030, 0.024)),
        origin=Origin(
            xyz=(0.0, -0.056, 0.046),
            rpy=(-0.16, 0.0, 0.0),
        ),
        material=materials["velvet"],
    )
    body.visual(
        Box((0.052, 0.016, 0.010)),
        origin=Origin(
            xyz=(-0.104, 0.022, 0.050),
        ),
        material=materials["steel"],
    )
    body.visual(
        Box((0.026, 0.010, 0.006)),
        origin=Origin(
            xyz=(-0.074, 0.022, 0.032),
        ),
        material=materials["steel"],
    )
    body.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(
            xyz=(-0.061, 0.022, 0.067),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=materials["steel"],
    )
    body.visual(
        Box((0.090, 0.018, 0.006)),
        origin=Origin(
            xyz=(0.0, outer_d * 0.5 - 0.009, 0.044),
        ),
        material=materials["brass"],
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, base_h)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_frame_mesh,
        origin=Origin(xyz=(0.0, lid_y_offset, 0.0)),
        material=materials["walnut"],
    )
    lid.visual(
        lid_bezel_mesh,
        origin=Origin(xyz=(0.0, lid_y_offset, 0.0)),
        material=materials["brass"],
    )
    lid.visual(
        lid_glass_mesh,
        origin=Origin(xyz=(0.0, lid_y_offset, 0.0)),
        material=materials["glass"],
    )
    lid.visual(
        Box((lid_window_w + 0.004, lid_window_d + 0.004, 0.010)),
        origin=Origin(
            xyz=(0.0, lid_y_offset, 0.005),
        ),
        material=materials["velvet"],
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=1.2,
        origin=Origin(xyz=(0.0, lid_y_offset, lid_t * 0.5)),
    )

    winder_rotor = model.part("winder_rotor")
    winder_rotor.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    winder_rotor.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    winder_rotor.visual(
        Cylinder(radius=0.026, length=0.062),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["leather"],
    )
    winder_rotor.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.021, 0.0, 0.0)),
        material=materials["leather"],
    )
    winder_rotor.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=materials["leather"],
    )
    winder_rotor.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    winder_rotor.inertial = Inertial.from_geometry(
        Box((0.130, 0.085, 0.085)),
        mass=0.65,
        origin=Origin(),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(
            xyz=(0.0, -outer_d * 0.5 + hinge_inset, base_h + hinge_reveal),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "winder_spin",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="winder_rotor",
        origin=Origin(xyz=(-0.045, 0.022, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.6,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.02,
        reason="The watch cushion is intentionally displayed above a plush presentation pedestal, so the body-side drive mount sits slightly below the visible rotation axis.",
    )
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "body",
        "lid",
        reason="The closed lid sits on a very tight reveal and generated hulls around the rounded trim can be conservative at the seam.",
    )
    ctx.allow_overlap(
        "body",
        "winder_rotor",
        reason="The body-side drive cup intentionally nests tightly around the rotating watch cushion hub; generated convex hulls overestimate this coaxial interface and report tiny false-positive overlap volumes.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(lid_hinge=0.0, winder_spin=0.0):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.18)
        ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.004, max_penetration=0.008)
        ctx.expect_aabb_overlap("winder_rotor", "body", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("winder_rotor", "body", axes="xy", max_dist=0.055)

    with ctx.pose(lid_hinge=1.18, winder_spin=0.0):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.07)
        ctx.expect_origin_distance("lid", "body", axes="xy", max_dist=0.12)
        ctx.expect_aabb_overlap("winder_rotor", "body", axes="xy", min_overlap=0.05)

    with ctx.pose(lid_hinge=0.6, winder_spin=math.pi / 2.0):
        ctx.expect_aabb_overlap("winder_rotor", "body", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("winder_rotor", "body", axes="xy", max_dist=0.055)

    with ctx.pose(lid_hinge=1.0, winder_spin=math.pi):
        ctx.expect_aabb_overlap("winder_rotor", "body", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("winder_rotor", "body", axes="xy", max_dist=0.055)
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
