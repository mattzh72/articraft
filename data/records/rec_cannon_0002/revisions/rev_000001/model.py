from __future__ import annotations

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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

REST_ELEVATION = -0.82
TRUNNION_Y = 0.146
TRUNNION_Z = 0.168


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _extrude_xz(profile: list[tuple[float, float]], thickness: float, name: str):
    return _save_mesh(
        name,
        ExtrudeGeometry.centered(profile, thickness).rotate_x(math.pi / 2.0),
    )


def _barrel_shell_mesh():
    return _save_mesh(
        "barrel_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.128, -0.082),
                (0.126, -0.066),
                (0.118, -0.030),
                (0.110, 0.018),
                (0.101, 0.104),
                (0.106, 0.152),
            ],
            [
                (0.0, -0.078),
                (0.046, -0.068),
                (0.058, -0.028),
                (0.064, 0.048),
                (0.070, 0.152),
            ],
            segments=72,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
    )


def _side_plate_mesh():
    return _extrude_xz(
        [
            (-0.168, 0.024),
            (0.086, 0.024),
            (0.114, 0.082),
            (0.114, 0.158),
            (0.020, 0.184),
            (-0.112, 0.152),
            (-0.168, 0.094),
        ],
        0.018,
        "side_plate.obj",
    )


def _wedge_body_mesh():
    return _extrude_xz(
        [
            (-0.060, 0.000),
            (0.056, 0.000),
            (0.056, 0.048),
            (-0.060, 0.014),
        ],
        0.048,
        "elevation_wedge.obj",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="siege_mortar", assets=ASSETS)

    oak = model.material("oak", rgba=(0.42, 0.28, 0.16, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.29, 0.19, 0.11, 1.0))
    iron = model.material("iron", rgba=(0.23, 0.24, 0.26, 1.0))
    bronze = model.material("bronze", rgba=(0.55, 0.42, 0.20, 1.0))
    black_iron = model.material("black_iron", rgba=(0.10, 0.10, 0.11, 1.0))

    bed = model.part("bed")
    bed.visual(Box((0.460, 0.060, 0.040)), origin=Origin(xyz=(0.0, -0.115, 0.038)), material=oak, name="left_rail")
    bed.visual(Box((0.460, 0.060, 0.040)), origin=Origin(xyz=(0.0, 0.115, 0.038)), material=oak, name="right_rail")
    bed.visual(Box((0.050, 0.320, 0.048)), origin=Origin(xyz=(0.180, 0.0, 0.040)), material=oak, name="front_transom")
    bed.visual(Box((0.042, 0.236, 0.020)), origin=Origin(xyz=(-0.184, 0.0, 0.010)), material=dark_oak, name="rear_transom")
    bed.visual(Box((0.324, 0.092, 0.012)), origin=Origin(xyz=(-0.002, 0.0, 0.006)), material=dark_oak, name="slot_floor")
    bed.visual(Box((0.324, 0.022, 0.024)), origin=Origin(xyz=(-0.002, -0.055, 0.018)), material=dark_oak, name="left_slot_guide")
    bed.visual(Box((0.324, 0.022, 0.024)), origin=Origin(xyz=(-0.002, 0.055, 0.018)), material=dark_oak, name="right_slot_guide")
    bed.visual(Box((0.160, 0.016, 0.016)), origin=Origin(xyz=(0.008, -0.037, 0.036)), material=oak, name="left_dovetail_lip")
    bed.visual(Box((0.160, 0.016, 0.016)), origin=Origin(xyz=(0.008, 0.037, 0.036)), material=oak, name="right_dovetail_lip")
    bed.visual(Box((0.064, 0.064, 0.036)), origin=Origin(xyz=(0.184, -0.115, 0.018)), material=dark_oak, name="front_left_foot")
    bed.visual(Box((0.064, 0.064, 0.036)), origin=Origin(xyz=(0.184, 0.115, 0.018)), material=dark_oak, name="front_right_foot")
    bed.visual(Box((0.064, 0.064, 0.036)), origin=Origin(xyz=(-0.184, -0.115, 0.018)), material=dark_oak, name="rear_left_foot")
    bed.visual(Box((0.064, 0.064, 0.036)), origin=Origin(xyz=(-0.184, 0.115, 0.018)), material=dark_oak, name="rear_right_foot")
    bed.visual(_side_plate_mesh(), origin=Origin(xyz=(-0.016, -0.188, 0.0)), material=oak, name="left_side_plate")
    bed.visual(_side_plate_mesh(), origin=Origin(xyz=(-0.016, 0.188, 0.0)), material=oak, name="right_side_plate")
    bed.visual(Box((0.092, 0.034, 0.112)), origin=Origin(xyz=(-0.008, -0.162, 0.080)), material=dark_oak, name="left_cheek_block")
    bed.visual(Box((0.092, 0.034, 0.112)), origin=Origin(xyz=(-0.008, 0.162, 0.080)), material=dark_oak, name="right_cheek_block")
    bed.visual(Box((0.084, 0.040, 0.018)), origin=Origin(xyz=(-0.006, -0.152, 0.143)), material=iron, name="left_trunnion_seat")
    bed.visual(Box((0.084, 0.040, 0.018)), origin=Origin(xyz=(-0.006, 0.152, 0.143)), material=iron, name="right_trunnion_seat")
    bed.visual(
        Box((0.120, 0.028, 0.044)),
        origin=Origin(xyz=(-0.112, -0.112, 0.086), rpy=(0.0, 0.42, 0.0)),
        material=dark_oak,
        name="left_rear_brace",
    )
    bed.visual(
        Box((0.120, 0.028, 0.044)),
        origin=Origin(xyz=(-0.112, 0.112, 0.086), rpy=(0.0, 0.42, 0.0)),
        material=dark_oak,
        name="right_rear_brace",
    )
    bed.visual(
        Box((0.112, 0.026, 0.040)),
        origin=Origin(xyz=(0.102, -0.112, 0.078), rpy=(0.0, -0.34, 0.0)),
        material=dark_oak,
        name="left_front_brace",
    )
    bed.visual(
        Box((0.112, 0.026, 0.040)),
        origin=Origin(xyz=(0.102, 0.112, 0.078), rpy=(0.0, -0.34, 0.0)),
        material=dark_oak,
        name="right_front_brace",
    )

    barrel = model.part("barrel")
    barrel.visual(
        _barrel_shell_mesh(),
        origin=Origin(xyz=(0.035, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.108, length=0.020),
        origin=Origin(xyz=(0.180, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.048, length=0.040),
        origin=Origin(xyz=(-0.058, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(-0.088, 0.0, 0.025)),
        material=iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.016, length=0.048),
        origin=Origin(xyz=(0.000, -TRUNNION_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.016, length=0.048),
        origin=Origin(xyz=(0.000, TRUNNION_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="right_trunnion",
    )
    barrel.visual(
        Box((0.042, 0.018, 0.050)),
        origin=Origin(xyz=(0.000, -0.121, 0.0)),
        material=bronze,
        name="left_trunnion_pad",
    )
    barrel.visual(
        Box((0.042, 0.018, 0.050)),
        origin=Origin(xyz=(0.000, 0.121, 0.0)),
        material=bronze,
        name="right_trunnion_pad",
    )
    barrel.visual(
        Box((0.078, 0.050, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, -0.082)),
        material=iron,
        name="breech_shoe",
    )

    wedge = model.part("elevation_wedge")
    wedge.visual(Box((0.112, 0.082, 0.012)), origin=Origin(xyz=(-0.002, 0.0, 0.006)), material=dark_oak, name="wedge_tongue")
    wedge.visual(_wedge_body_mesh(), material=oak, name="wedge_body")
    wedge.visual(Box((0.032, 0.050, 0.014)), origin=Origin(xyz=(-0.054, 0.0, 0.012)), material=black_iron, name="wedge_pull")

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=barrel,
        origin=Origin(xyz=(-0.006, 0.0, TRUNNION_Z), rpy=(0.0, REST_ELEVATION, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.8, lower=0.0, upper=0.22),
    )
    model.articulation(
        "wedge_slide",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=wedge,
        origin=Origin(xyz=(-0.012, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.050),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    bed = object_model.get_part("bed")
    barrel = object_model.get_part("barrel")
    wedge = object_model.get_part("elevation_wedge")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    wedge_slide = object_model.get_articulation("wedge_slide")

    barrel_shell = barrel.get_visual("barrel_shell")
    muzzle_band = barrel.get_visual("muzzle_band")
    left_trunnion = barrel.get_visual("left_trunnion")
    right_trunnion = barrel.get_visual("right_trunnion")
    breech_shoe = barrel.get_visual("breech_shoe")

    left_trunnion_seat = bed.get_visual("left_trunnion_seat")
    right_trunnion_seat = bed.get_visual("right_trunnion_seat")
    front_transom = bed.get_visual("front_transom")
    left_side_plate = bed.get_visual("left_side_plate")
    left_slot_guide = bed.get_visual("left_slot_guide")
    right_slot_guide = bed.get_visual("right_slot_guide")
    slot_floor = bed.get_visual("slot_floor")
    left_front_foot = bed.get_visual("front_left_foot")
    right_front_foot = bed.get_visual("front_right_foot")
    left_rear_foot = bed.get_visual("rear_left_foot")
    right_rear_foot = bed.get_visual("rear_right_foot")

    wedge_tongue = wedge.get_visual("wedge_tongue")
    wedge_body = wedge.get_visual("wedge_body")
    wedge_pull = wedge.get_visual("wedge_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.130)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        barrel,
        bed,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=left_trunnion,
        negative_elem=left_trunnion_seat,
        name="left_trunnion_bears_on_cradle",
    )
    ctx.expect_gap(
        barrel,
        bed,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=right_trunnion,
        negative_elem=right_trunnion_seat,
        name="right_trunnion_bears_on_cradle",
    )
    ctx.expect_overlap(
        barrel,
        bed,
        axes="y",
        min_overlap=0.032,
        elem_a=left_trunnion,
        elem_b=left_trunnion_seat,
        name="left_trunnion_reads_as_cradle_journal",
    )
    ctx.expect_overlap(
        barrel,
        bed,
        axes="y",
        min_overlap=0.032,
        elem_a=right_trunnion,
        elem_b=right_trunnion_seat,
        name="right_trunnion_reads_as_cradle_journal",
    )
    ctx.expect_gap(
        wedge,
        bed,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.001,
        positive_elem=wedge_tongue,
        negative_elem=slot_floor,
        name="wedge_rides_on_slot_floor",
    )
    ctx.expect_gap(
        bed,
        wedge,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem=right_slot_guide,
        negative_elem=wedge_tongue,
        name="wedge_tongue_clears_right_guide",
    )
    ctx.expect_gap(
        wedge,
        bed,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem=wedge_tongue,
        negative_elem=left_slot_guide,
        name="wedge_tongue_clears_left_guide",
    )
    ctx.expect_overlap(
        barrel,
        wedge,
        axes="x",
        min_overlap=0.028,
        elem_a=breech_shoe,
        elem_b=wedge_body,
        name="wedge_sits_under_breech",
    )
    ctx.expect_gap(
        barrel,
        wedge,
        axis="z",
        max_gap=0.010,
        max_penetration=0.002,
        positive_elem=breech_shoe,
        negative_elem=wedge_body,
        name="wedge_supports_breech",
    )
    ctx.expect_gap(
        barrel,
        bed,
        axis="z",
        min_gap=0.170,
        positive_elem=muzzle_band,
        negative_elem=front_transom,
        name="muzzle_stands_high_above_low_bed",
    )
    ctx.expect_overlap(
        barrel,
        bed,
        axes="x",
        min_overlap=0.100,
        elem_a=barrel_shell,
        elem_b=left_side_plate,
        name="short_barrel_spans_side_plates",
    )
    ctx.expect_overlap(
        bed,
        bed,
        axes="y",
        min_overlap=0.050,
        elem_a=left_front_foot,
        elem_b=left_rear_foot,
        name="left_feet_make_ground_mount_stance",
    )
    ctx.expect_overlap(
        bed,
        bed,
        axes="y",
        min_overlap=0.050,
        elem_a=right_front_foot,
        elem_b=right_rear_foot,
        name="right_feet_make_ground_mount_stance",
    )
    ctx.expect_overlap(
        wedge,
        wedge,
        axes="x",
        min_overlap=0.010,
        elem_a=wedge_tongue,
        elem_b=wedge_pull,
        name="wedge_has_pull_handle",
    )
    with ctx.pose({barrel_elevation: 0.18, wedge_slide: 0.040}):
        ctx.expect_gap(
            barrel,
            bed,
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
            positive_elem=left_trunnion,
            negative_elem=left_trunnion_seat,
            name="left_trunnion_stays_seated_when_elevated",
        )
        ctx.expect_overlap(
            barrel,
            wedge,
            axes="x",
            min_overlap=0.022,
            elem_a=breech_shoe,
            elem_b=wedge_body,
            name="advanced_wedge_stays_under_breech",
        )
        ctx.expect_gap(
            barrel,
            wedge,
            axis="z",
            max_gap=0.012,
            max_penetration=0.002,
            positive_elem=breech_shoe,
            negative_elem=wedge_body,
            name="advanced_wedge_still_supports_breech",
        )
        ctx.expect_gap(
            barrel,
            bed,
            axis="z",
            min_gap=0.200,
            positive_elem=muzzle_band,
            negative_elem=front_transom,
            name="raised_muzzle_climbs_further_above_front_transom",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
