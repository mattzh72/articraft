from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


HOUSING_W = 0.380
HOUSING_D = 0.380
HOUSING_T = 0.026

PAD_COUNT = 8
PAD_PITCH = 0.035
PAD_SIZE = 0.026
PAD_WELL = 0.029
PAD_H = 0.007
PAD_STEM = 0.005
PAD_TRAVEL = 0.004
PAD_GRID_CENTER_Y = 0.050

FADER_COUNT = 8
FADER_SLOT_L = 0.032
FADER_SLOT_W = 0.008
FADER_CAP_L = 0.014
FADER_CAP_W = 0.020
FADER_CAP_H = 0.010
FADER_STEM_L = 0.006
FADER_STEM_W = FADER_SLOT_W
FADER_TRAVEL = 0.007
FADER_Y = -0.145


def _grid_positions() -> list[tuple[float, float]]:
    start = -((PAD_COUNT - 1) * PAD_PITCH) / 2.0
    return [
        (start + col * PAD_PITCH, PAD_GRID_CENTER_Y + start + row * PAD_PITCH)
        for row in range(PAD_COUNT)
        for col in range(PAD_COUNT)
    ]


def _fader_positions() -> list[tuple[float, float]]:
    start = -((FADER_COUNT - 1) * 0.043) / 2.0
    return [(start + idx * 0.043, FADER_Y) for idx in range(FADER_COUNT)]


def _housing_geometry() -> cq.Workplane:
    housing = cq.Workplane("XY").box(HOUSING_W, HOUSING_D, HOUSING_T)
    housing = housing.edges("|Z").fillet(0.018)

    pad_cutters = (
        cq.Workplane("XY")
        .pushPoints(_grid_positions())
        .box(PAD_WELL, PAD_WELL, HOUSING_T * 3.0)
    )
    slot_cutters = (
        cq.Workplane("XY")
        .pushPoints(_fader_positions())
        .box(FADER_SLOT_L, FADER_SLOT_W, HOUSING_T * 3.0)
    )
    housing = housing.cut(pad_cutters).cut(slot_cutters)
    return housing


def _pad_cap_geometry() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PAD_SIZE, PAD_SIZE, PAD_H)
        .edges("|Z")
        .fillet(0.003)
        .edges(">Z")
        .fillet(0.001)
    )


def _fader_cap_geometry() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FADER_CAP_L, FADER_CAP_W, FADER_CAP_H)
        .edges("|Z")
        .fillet(0.0025)
        .edges(">Z")
        .fillet(0.001)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grid_midi_pad_controller")

    mat_shell = model.material("satin_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    mat_pad = model.material("smoky_translucent_pad", rgba=(0.58, 0.62, 0.70, 0.92))
    mat_pad_alt = model.material("cool_accent_pad", rgba=(0.20, 0.42, 0.80, 0.92))
    mat_fader = model.material("light_gray_fader_caps", rgba=(0.78, 0.78, 0.74, 1.0))

    housing_mesh = mesh_from_cadquery(_housing_geometry(), "controller_housing", tolerance=0.0006)
    pad_mesh = mesh_from_cadquery(_pad_cap_geometry(), "rounded_pad_cap", tolerance=0.0004)
    fader_mesh = mesh_from_cadquery(_fader_cap_geometry(), "fader_thumb_cap", tolerance=0.0004)

    housing = model.part("housing")
    housing.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_T / 2.0)),
        material=mat_shell,
        name="top_shell",
    )

    pad_limits = MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=PAD_TRAVEL)
    pad_positions = _grid_positions()
    for row in range(PAD_COUNT):
        for col in range(PAD_COUNT):
            idx = row * PAD_COUNT + col
            x, y = pad_positions[idx]
            pad = model.part(f"pad_{row}_{col}")
            pad_material = mat_pad_alt if (row, col) in {(0, 0), (0, 7), (7, 0), (7, 7)} else mat_pad
            pad.visual(
                pad_mesh,
                origin=Origin(xyz=(0.0, 0.0, PAD_H / 2.0)),
                material=pad_material,
                name="pad_cap",
            )
            pad.visual(
                Box((PAD_WELL, PAD_WELL, PAD_STEM)),
                origin=Origin(xyz=(0.0, 0.0, -PAD_STEM / 2.0 + 0.00015)),
                material=mat_pad,
                name="guide_plunger",
            )
            model.articulation(
                f"housing_to_pad_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=pad,
                origin=Origin(xyz=(x, y, HOUSING_T + 0.00025)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=pad_limits,
            )

    fader_limits = MotionLimits(effort=6.0, velocity=0.18, lower=-FADER_TRAVEL, upper=FADER_TRAVEL)
    for idx, (x, y) in enumerate(_fader_positions()):
        fader = model.part(f"fader_{idx}")
        fader.visual(
            fader_mesh,
            origin=Origin(xyz=(0.0, 0.0, FADER_CAP_H / 2.0 + 0.0008)),
            material=mat_fader,
            name="fader_cap",
        )
        fader.visual(
            Box((FADER_STEM_L, FADER_STEM_W, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            material=mat_fader,
            name="slot_stem",
        )
        model.articulation(
            f"housing_to_fader_{idx}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, y, HOUSING_T + 0.0004)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=fader_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    first_pad = object_model.get_part("pad_0_0")
    first_pad_joint = object_model.get_articulation("housing_to_pad_0_0")
    first_fader = object_model.get_part("fader_0")
    first_fader_joint = object_model.get_articulation("housing_to_fader_0")

    pad_parts = [p for p in object_model.parts if p.name.startswith("pad_")]
    fader_parts = [p for p in object_model.parts if p.name.startswith("fader_")]
    pad_joints = [j for j in object_model.articulations if j.name.startswith("housing_to_pad_")]
    fader_joints = [j for j in object_model.articulations if j.name.startswith("housing_to_fader_")]

    ctx.check("has sixty four compressing pads", len(pad_parts) == 64 and len(pad_joints) == 64)
    ctx.check("has bottom row faders", len(fader_parts) == FADER_COUNT and len(fader_joints) == FADER_COUNT)
    ctx.expect_gap(
        first_pad,
        housing,
        axis="z",
        positive_elem="pad_cap",
        negative_elem="top_shell",
        max_gap=0.001,
        max_penetration=0.0002,
        name="pad cap sits flush above housing",
    )
    ctx.expect_overlap(
        first_pad,
        housing,
        axes="xy",
        elem_a="guide_plunger",
        elem_b="top_shell",
        min_overlap=0.010,
        name="pad plunger is retained in its opening",
    )

    rest_pad_position = ctx.part_world_position(first_pad)
    with ctx.pose({first_pad_joint: PAD_TRAVEL}):
        pressed_pad_position = ctx.part_world_position(first_pad)
    ctx.check(
        "pad compresses downward",
        rest_pad_position is not None
        and pressed_pad_position is not None
        and pressed_pad_position[2] < rest_pad_position[2] - 0.003,
        details=f"rest={rest_pad_position}, pressed={pressed_pad_position}",
    )

    rest_fader_position = ctx.part_world_position(first_fader)
    with ctx.pose({first_fader_joint: FADER_TRAVEL}):
        slid_fader_position = ctx.part_world_position(first_fader)
    ctx.check(
        "fader slides horizontally",
        rest_fader_position is not None
        and slid_fader_position is not None
        and slid_fader_position[0] > rest_fader_position[0] + 0.005,
        details=f"rest={rest_fader_position}, slid={slid_fader_position}",
    )

    return ctx.report()


object_model = build_object_model()
