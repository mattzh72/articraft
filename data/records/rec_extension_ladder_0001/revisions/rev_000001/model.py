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
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MAX_EXTENSION = 1.40


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    for kwargs in (
        {"name": name, "rgba": rgba},
        {"name": name, "color": rgba},
        {"name": name, "diffuse": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name=name)


def _add_channel_rail(
    part,
    *,
    x_center: float,
    y_center: float,
    z_center: float,
    length: float,
    web_offset: float,
    web_thickness: float,
    rail_depth: float,
    flange_width: float,
    flange_thickness: float,
    flange_separation: float,
    inner_lip_center: float,
    inner_lip_width: float,
    inner_lip_depth: float,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    material,
) -> None:
    for side in (-1.0, 1.0):
        part.visual(
            Box((web_thickness, rail_depth, length)),
            origin=Origin(
                xyz=(
                    side * (x_center + web_offset) + x_shift,
                    y_center + y_shift,
                    z_center,
                )
            ),
            material=material,
        )
        for flange_sign in (-1.0, 1.0):
            part.visual(
                Box((flange_width, flange_thickness, length)),
                origin=Origin(
                    xyz=(
                        side * x_center + x_shift,
                        y_center + y_shift + flange_sign * (flange_separation / 2.0),
                        z_center,
                    )
                ),
                material=material,
            )
        part.visual(
            Box((inner_lip_width, inner_lip_depth, length)),
            origin=Origin(xyz=(side * inner_lip_center + x_shift, y_center + y_shift, z_center)),
            material=material,
        )


def _add_rungs(
    part,
    *,
    z_positions: list[float],
    y_center: float,
    radius: float,
    length: float,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    material,
) -> None:
    for z in z_positions:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(x_shift, y_center + y_shift, z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder", assets=ASSETS)

    aluminum = _make_material("brushed_aluminum", (0.82, 0.84, 0.86, 1.0))
    steel = _make_material("zinc_steel", (0.62, 0.64, 0.67, 1.0))
    rubber = _make_material("black_rubber", (0.08, 0.08, 0.09, 1.0))
    nylon = _make_material("nylon_glide", (0.13, 0.14, 0.15, 1.0))
    rope = _make_material("tan_rope", (0.70, 0.58, 0.34, 1.0))
    model.materials = [aluminum, steel, rubber, nylon, rope]

    base_length = 3.72
    fly_length = 3.08
    joint_z = 1.78
    fly_bottom_local = -1.40
    fly_top_local = fly_bottom_local + fly_length
    fly_center_local = 0.5 * (fly_bottom_local + fly_top_local)
    fly_frame_x = 0.198
    fly_frame_y = 0.030
    fly_local_shift_x = -fly_frame_x
    fly_local_shift_y = 0.021 - fly_frame_y

    base = model.part("base_section")
    _add_channel_rail(
        base,
        x_center=0.215,
        y_center=0.0,
        z_center=base_length / 2.0,
        length=base_length,
        web_offset=0.014,
        web_thickness=0.006,
        rail_depth=0.026,
        flange_width=0.034,
        flange_thickness=0.004,
        flange_separation=0.022,
        inner_lip_center=0.202,
        inner_lip_width=0.008,
        inner_lip_depth=0.010,
        material=aluminum,
    )
    _add_rungs(
        base,
        z_positions=[0.30 + 0.296 * i for i in range(12)],
        y_center=0.002,
        radius=0.016,
        length=0.396,
        material=aluminum,
    )
    for side in (-1.0, 1.0):
        base.visual(
            Box((0.052, 0.032, 0.040)),
            origin=Origin(xyz=(side * 0.215, 0.0, 0.020)),
            material=rubber,
        )
        base.visual(
            Box((0.056, 0.012, 0.014)),
            origin=Origin(xyz=(side * 0.215, 0.010, 0.007)),
            material=rubber,
        )
        base.visual(
            Box((0.040, 0.028, 0.032)),
            origin=Origin(xyz=(side * 0.215, 0.0, base_length - 0.016)),
            material=nylon,
        )
        base.visual(
            Box((0.022, 0.012, 0.050)),
            origin=Origin(xyz=(side * 0.202, 0.018, base_length - 0.045)),
            material=nylon,
        )
        base.visual(
            Box((0.008, 0.020, 2.56)),
            origin=Origin(xyz=(side * 0.198, 0.017, 2.28)),
            material=nylon,
        )
    base.visual(
        Box((0.014, 0.040, 0.200)),
        origin=Origin(xyz=(0.238, 0.020, 3.42)),
        material=steel,
    )
    base.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.244, 0.027, 3.52), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    base.visual(
        Cylinder(radius=0.004, length=3.06),
        origin=Origin(xyz=(0.247, 0.027, 1.98)),
        material=rope,
    )
    base.visual(
        Box((0.018, 0.038, 0.060)),
        origin=Origin(xyz=(0.238, 0.020, 0.66)),
        material=steel,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.50, 0.11, base_length)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.01, base_length / 2.0)),
    )

    fly = model.part("fly_section")
    _add_channel_rail(
        fly,
        x_center=0.184,
        y_center=0.030,
        z_center=fly_center_local,
        length=fly_length,
        web_offset=0.012,
        web_thickness=0.005,
        rail_depth=0.022,
        flange_width=0.030,
        flange_thickness=0.0035,
        flange_separation=0.018,
        inner_lip_center=0.173,
        inner_lip_width=0.007,
        inner_lip_depth=0.008,
        x_shift=fly_local_shift_x,
        y_shift=fly_local_shift_y,
        material=aluminum,
    )
    _add_rungs(
        fly,
        z_positions=[-1.12 + 0.28 * i for i in range(10)],
        y_center=0.030,
        radius=0.014,
        length=0.338,
        x_shift=fly_local_shift_x,
        y_shift=fly_local_shift_y,
        material=aluminum,
    )
    for side in (-1.0, 1.0):
        fly.visual(
            Box((0.034, 0.024, 0.032)),
            origin=Origin(
                xyz=(
                    side * 0.184 + fly_local_shift_x,
                    0.030 + fly_local_shift_y,
                    fly_top_local - 0.016,
                )
            ),
            material=nylon,
        )
        for shoe_z in (-1.16, 1.34):
            fly.visual(
                Box((0.034, 0.012, 0.140)),
                origin=Origin(
                    xyz=(side * 0.184 + fly_local_shift_x, 0.023 + fly_local_shift_y, shoe_z)
                ),
                material=nylon,
            )
        fly.visual(
            Box((0.010, 0.024, 0.150)),
            origin=Origin(xyz=(side * 0.203 + fly_local_shift_x, 0.030 + fly_local_shift_y, -0.62)),
            material=steel,
        )
        fly.visual(
            Box((0.020, 0.010, 0.100)),
            origin=Origin(xyz=(side * 0.191 + fly_local_shift_x, 0.021 + fly_local_shift_y, -0.69)),
            material=steel,
        )
        fly.visual(
            Box((0.024, 0.008, 0.040)),
            origin=Origin(xyz=(side * 0.186 + fly_local_shift_x, 0.020 + fly_local_shift_y, -0.75)),
            material=steel,
        )
    fly.visual(
        Box((0.020, 0.012, 0.080)),
        origin=Origin(xyz=(0.006, 0.004, 0.000)),
        material=steel,
    )
    fly.visual(
        Box((0.018, 0.024, 0.060)),
        origin=Origin(xyz=(0.206 + fly_local_shift_x, 0.030 + fly_local_shift_y, 1.46)),
        material=steel,
    )
    fly.inertial = Inertial.from_geometry(
        Box((0.43, 0.10, fly_length)),
        mass=9.0,
        origin=Origin(xyz=(fly_local_shift_x, 0.021 + fly_local_shift_y, fly_center_local)),
    )

    model.articulation(
        "section_extend",
        ArticulationType.PRISMATIC,
        parent="base_section",
        child="fly_section",
        origin=Origin(xyz=(fly_frame_x, fly_frame_y, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=MAX_EXTENSION,
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
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "section_extend",
        "fly_section",
        world_axis="z",
        direction="positive",
        min_delta=0.20,
    )

    with ctx.pose(section_extend=0.0):
        ctx.expect_aabb_overlap("fly_section", "base_section", axes="xy", min_overlap=0.01)

    with ctx.pose(section_extend=0.70):
        ctx.expect_aabb_overlap("fly_section", "base_section", axes="xy", min_overlap=0.01)

    with ctx.pose(section_extend=MAX_EXTENSION):
        ctx.expect_aabb_overlap("fly_section", "base_section", axes="xy", min_overlap=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
