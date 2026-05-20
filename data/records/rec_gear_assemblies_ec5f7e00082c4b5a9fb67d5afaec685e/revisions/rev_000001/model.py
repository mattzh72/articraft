from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_T = 0.016
BASE_DEPTH = 0.11
BASE_WIDTH = 0.34
FRAME_T = 0.020
PLATE_W = 0.34
PLATE_H = 0.25
PLATE_EMBED = 0.006
RIB_T = 0.012
GEAR_MODULE = 0.006
GEAR_WIDTH = 0.016
GEAR_START_X = 0.046
BOSS_BASE_LEN = 0.012
BOSS_CAP_LEN = 0.006
REAR_BOSS_LEN = 0.006
BOSS_FRONT_X = FRAME_T / 2.0 + BOSS_BASE_LEN + BOSS_CAP_LEN
SHAFT_Z = 0.182
INPUT_TO_IDLER = 0.110
IDLER_TO_OUTPUT = 0.080

SHAFT_SPECS = {
    "input_shaft": {
        "y": -INPUT_TO_IDLER,
        "z": SHAFT_Z,
        "shaft_r": 0.007,
        "collar_r": 0.018,
        "end_r": 0.012,
        "teeth": 22,
        "hub_r": 0.020,
        "phase_deg": 0.0,
    },
    "idler_shaft": {
        "y": 0.0,
        "z": SHAFT_Z,
        "shaft_r": 0.006,
        "collar_r": 0.016,
        "end_r": 0.0105,
        "teeth": 14,
        "hub_r": 0.017,
        "phase_deg": 180.0 / 14.0,
    },
    "output_shaft": {
        "y": IDLER_TO_OUTPUT,
        "z": SHAFT_Z,
        "shaft_r": 0.0055,
        "collar_r": 0.015,
        "end_r": 0.010,
        "teeth": 12,
        "hub_r": 0.016,
        "phase_deg": 0.0,
    },
}


def cyl_x(radius: float, length: float, x0: float) -> cq.Shape:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, 0.0)).val()


def fuse_all(*solids: cq.Shape) -> cq.Shape:
    result = solids[0]
    for solid in solids[1:]:
        result = result.fuse(solid)
    return result


def shaft_hole_radius(shaft_r: float) -> float:
    return shaft_r + 0.0006


def boss_radius(shaft_r: float) -> float:
    return max(shaft_r * 3.2, 0.018)


def build_frame_shape() -> cq.Shape:
    base = (
        cq.Workplane("XY")
        .center(-(BASE_DEPTH / 2.0) + 0.02, 0.0)
        .rect(BASE_DEPTH, BASE_WIDTH)
        .extrude(BASE_T)
        .val()
    )

    plate = (
        cq.Workplane("YZ")
        .center(0.0, BASE_T - PLATE_EMBED + PLATE_H / 2.0)
        .rect(PLATE_W, PLATE_H)
        .extrude(FRAME_T / 2.0, both=True)
        .val()
    )

    rib_profile = [
        (-BASE_DEPTH + 0.024, BASE_T),
        (-0.010, BASE_T),
        (-0.010, 0.145),
    ]
    left_rib = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_T / 2.0, both=True)
        .translate((0.0, -0.115, 0.0))
        .val()
    )
    right_rib = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_T / 2.0, both=True)
        .translate((0.0, 0.105, 0.0))
        .val()
    )

    frame = fuse_all(base, plate, left_rib, right_rib)

    for spec in SHAFT_SPECS.values():
        y = spec["y"]
        z = spec["z"]
        r_outer = boss_radius(spec["shaft_r"])
        r_cap = r_outer * 0.76
        front_boss = cyl_x(r_outer, BOSS_BASE_LEN, FRAME_T / 2.0).translate((0.0, y, z))
        front_cap = cyl_x(r_cap, BOSS_CAP_LEN, FRAME_T / 2.0 + BOSS_BASE_LEN).translate((0.0, y, z))
        rear_pad = cyl_x(r_cap, REAR_BOSS_LEN, -FRAME_T / 2.0 - REAR_BOSS_LEN).translate((0.0, y, z))
        frame = fuse_all(frame, front_boss, front_cap, rear_pad)

    for y in (-0.055, 0.065):
        lightening_hole = cyl_x(0.030, FRAME_T + 0.020, -FRAME_T / 2.0 - 0.010).translate((0.0, y, 0.095))
        frame = frame.cut(lightening_hole)

    for spec in SHAFT_SPECS.values():
        hole = cyl_x(
            shaft_hole_radius(spec["shaft_r"]),
            FRAME_T + BOSS_BASE_LEN + BOSS_CAP_LEN + REAR_BOSS_LEN + 0.040,
            -REAR_BOSS_LEN - FRAME_T / 2.0 - 0.010,
        ).translate((0.0, spec["y"], spec["z"]))
        frame = frame.cut(hole)

    return frame


def build_shaft_visuals(part_name: str, spec: dict[str, float]) -> dict[str, cq.Shape]:
    shaft_r = spec["shaft_r"]
    collar_r = spec["collar_r"]
    hub_r = spec["hub_r"]
    end_r = spec["end_r"]
    shaft_length = 0.134
    shaft_back_x = -0.020
    collar_len = 0.006
    end_cap_len = 0.005
    end_cap_x = 0.106
    hub_back = 0.008
    hub_len = 0.026
    phase_deg = spec["phase_deg"]

    shaft = cyl_x(shaft_r, shaft_length, shaft_back_x)
    collar = cyl_x(collar_r, collar_len, BOSS_FRONT_X)
    end_cap = cyl_x(end_r, end_cap_len, end_cap_x)

    gear = cq.Workplane("YZ").gear(
        SpurGear(
            module=GEAR_MODULE,
            teeth_number=spec["teeth"],
            width=GEAR_WIDTH,
            bore_d=shaft_r * 2.0,
        )
    ).findSolid()
    gear = gear.translate((GEAR_START_X, 0.0, 0.0))
    hub = cyl_x(hub_r, hub_len, GEAR_START_X - hub_back)
    gear = gear.fuse(hub).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), phase_deg)

    return {
        "shaft": shaft,
        "collar": collar,
        "gear": gear,
        "end_cap": end_cap,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_spur_gear_train_bench")

    frame_color = model.material("frame_steel", color=(0.22, 0.25, 0.30, 1.0))
    shaft_color = model.material("shaft_steel", color=(0.73, 0.75, 0.78, 1.0))
    gear_color = model.material("gear_bronze", color=(0.74, 0.62, 0.30, 1.0))
    end_cap_color = model.material("retainer_dark", color=(0.16, 0.17, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(build_frame_shape(), "frame"),
        material=frame_color,
        name="frame_shell",
    )

    for part_name, spec in SHAFT_SPECS.items():
        shaft_part = model.part(part_name)
        visuals = build_shaft_visuals(part_name, spec)
        shaft_part.visual(
            mesh_from_cadquery(visuals["shaft"], f"{part_name}_shaft"),
            material=shaft_color,
            name="shaft",
        )
        shaft_part.visual(
            mesh_from_cadquery(visuals["collar"], f"{part_name}_collar"),
            material=shaft_color,
            name="collar",
        )
        shaft_part.visual(
            mesh_from_cadquery(visuals["gear"], f"{part_name}_gear"),
            material=gear_color,
            name="gear",
        )
        shaft_part.visual(
            mesh_from_cadquery(visuals["end_cap"], f"{part_name}_end_cap"),
            material=end_cap_color,
            name="end_cap",
        )

        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=shaft_part,
            origin=Origin(xyz=(0.0, spec["y"], spec["z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=12.0,
                lower=-2.0 * pi,
                upper=2.0 * pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    joints = [
        object_model.get_articulation("frame_to_input_shaft"),
        object_model.get_articulation("frame_to_idler_shaft"),
        object_model.get_articulation("frame_to_output_shaft"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for shaft in (input_shaft, idler_shaft, output_shaft):
        ctx.expect_contact(shaft, frame, elem_a="collar", name=f"{shaft.name}_supported_by_frame")
        ctx.expect_gap(
            shaft,
            frame,
            axis="x",
            min_gap=0.0095,
            max_gap=0.025,
            positive_elem="gear",
            name=f"{shaft.name}_gear_clear_of_frame",
        )

    axes_ok = all(
        isclose(joint.axis[0], 1.0, abs_tol=1e-9)
        and isclose(joint.axis[1], 0.0, abs_tol=1e-9)
        and isclose(joint.axis[2], 0.0, abs_tol=1e-9)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is not None
        and joint.motion_limits.upper is not None
        and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
        for joint in joints
    )
    ctx.check("shaft_joints_are_parallel_x_revolutes", axes_ok, "All three shafts should rotate about the shared X bearing axis.")

    positions = {
        shaft.name: ctx.part_world_position(shaft)
        for shaft in (input_shaft, idler_shaft, output_shaft)
    }
    spacing_ok = (
        positions["input_shaft"] is not None
        and positions["idler_shaft"] is not None
        and positions["output_shaft"] is not None
        and isclose(positions["input_shaft"][2], positions["idler_shaft"][2], abs_tol=1e-6)
        and isclose(positions["idler_shaft"][2], positions["output_shaft"][2], abs_tol=1e-6)
        and isclose(positions["idler_shaft"][1] - positions["input_shaft"][1], INPUT_TO_IDLER, abs_tol=1e-4)
        and isclose(positions["output_shaft"][1] - positions["idler_shaft"][1], IDLER_TO_OUTPUT, abs_tol=1e-4)
    )
    ctx.check(
        "gear_centers_are_coplanar_and_meshing_spaced",
        spacing_ok,
        "The three shaft centers should stay in one gear plane with the intended center distances.",
    )

    def elem_center_x(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return 0.5 * (min_corner[0] + max_corner[0])

    gear_center_xs = [elem_center_x(shaft, "gear") for shaft in (input_shaft, idler_shaft, output_shaft)]
    plane_ok = all(x is not None for x in gear_center_xs) and max(gear_center_xs) - min(gear_center_xs) <= 1e-4
    ctx.check(
        "gear_faces_share_one_mesh_plane",
        plane_ok,
        "All three spur gears should sit in the same exposed gear plane ahead of the frame.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
