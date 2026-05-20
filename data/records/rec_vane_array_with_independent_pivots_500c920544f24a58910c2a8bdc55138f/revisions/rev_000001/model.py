from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_flap_array_panel")

    frame_mat = Material("powder_coated_frame", color=(0.08, 0.09, 0.10, 1.0))
    rail_mat = Material("slightly_worn_black_rails", color=(0.13, 0.14, 0.15, 1.0))
    flap_mat = Material("painted_aluminum_flaps", color=(0.30, 0.43, 0.52, 1.0))
    hinge_mat = Material("brushed_steel_hinges", color=(0.70, 0.68, 0.62, 1.0))
    seal_mat = Material("dark_rubber_seals", color=(0.02, 0.022, 0.025, 1.0))

    frame = model.part("frame")

    # A stiff backing plate and raised rail grid make the object read as a real
    # service panel rather than a collection of floating doors.
    frame.visual(
        Box((1.20, 0.020, 0.82)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_mat,
        name="back_plate",
    )
    frame.visual(
        Box((1.20, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, -0.020, 0.375)),
        material=rail_mat,
        name="top_rail",
    )
    frame.visual(
        Box((1.20, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, -0.020, -0.375)),
        material=rail_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.070, 0.028, 0.82)),
        origin=Origin(xyz=(-0.565, -0.020, 0.0)),
        material=rail_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.070, 0.028, 0.82)),
        origin=Origin(xyz=(0.565, -0.020, 0.0)),
        material=rail_mat,
        name="side_rail_1",
    )
    for index, x in enumerate((-0.18, 0.18)):
        frame.visual(
            Box((0.038, 0.030, 0.72)),
            origin=Origin(xyz=(x, -0.023, 0.0)),
            material=rail_mat,
            name=f"vertical_mullion_{index}",
        )
    frame.visual(
        Box((1.08, 0.030, 0.058)),
        origin=Origin(xyz=(0.0, -0.023, -0.006)),
        material=rail_mat,
        name="middle_rail",
    )

    flap_width = 0.292
    flap_height = 0.235
    hinge_y = -0.047
    hinge_radius = 0.012
    hinge_z_by_row = (0.275, -0.075)
    column_x = (-0.36, 0.0, 0.36)

    for row, hinge_z in enumerate(hinge_z_by_row):
        support_top_z = 0.357 if row == 0 else 0.010
        support_mid_z = (hinge_z + support_top_z) * 0.5
        support_height = abs(support_top_z - hinge_z) + 0.022
        for col, cx in enumerate(column_x):
            # Two fixed hinge stanchions and a continuous captured pin define a
            # separate hinge axis for each flap in the repeated array.
            for side, sign in (("a", -1.0), ("b", 1.0)):
                sx = cx + sign * (flap_width * 0.50 + 0.018)
                frame.visual(
                    Box((0.020, 0.034, support_height)),
                    origin=Origin(xyz=(sx, hinge_y, support_mid_z)),
                    material=hinge_mat,
                    name=f"stanchion_{row}_{col}_{side}",
                )
            frame.visual(
                Cylinder(radius=0.0065, length=flap_width + 0.070),
                origin=Origin(xyz=(cx, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_mat,
                name=f"hinge_pin_{row}_{col}",
            )

            # A thin gasket line in each cell visually separates the moving flap
            # from the stationary panel surface.
            frame.visual(
                Box((flap_width + 0.030, 0.006, 0.014)),
                origin=Origin(xyz=(cx, -0.034, hinge_z - flap_height - 0.021)),
                material=seal_mat,
                name=f"bottom_seal_{row}_{col}",
            )

            flap = model.part(f"flap_{row}_{col}")
            flap.visual(
                Box((flap_width, 0.012, flap_height)),
                origin=Origin(xyz=(0.0, -0.016, -hinge_radius - flap_height * 0.5)),
                material=flap_mat,
                name="face",
            )
            flap.visual(
                Cylinder(radius=hinge_radius, length=flap_width * 0.70),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_mat,
                name="barrel",
            )
            flap.visual(
                Box((flap_width * 0.84, 0.007, 0.042)),
                origin=Origin(xyz=(0.0, -0.013, -0.020)),
                material=hinge_mat,
                name="hinge_leaf",
            )
            flap.visual(
                Box((flap_width * 0.82, 0.010, 0.018)),
                origin=Origin(xyz=(0.0, -0.024, -flap_height + 0.008)),
                material=seal_mat,
                name="lower_lip",
            )
            for side, sign in (("edge_0", -1.0), ("edge_1", 1.0)):
                flap.visual(
                    Box((0.014, 0.010, flap_height * 0.72)),
                    origin=Origin(
                        xyz=(sign * (flap_width * 0.5 - 0.024), -0.024, -flap_height * 0.54)
                    ),
                    material=flap_mat,
                    name=side,
                )
            flap.visual(
                Box((0.060, 0.012, 0.030)),
                origin=Origin(xyz=(0.0, -0.027, -flap_height * 0.58)),
                material=hinge_mat,
                name="pull_tab",
            )

            model.articulation(
                f"frame_to_flap_{row}_{col}",
                ArticulationType.REVOLUTE,
                parent=frame,
                child=flap,
                origin=Origin(xyz=(cx, hinge_y, hinge_z)),
                axis=(-1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.10),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    joint_names = [f"frame_to_flap_{row}_{col}" for row in range(2) for col in range(3)]

    ctx.check(
        "six independent flap hinge axes",
        len(joint_names) == 6
        and all(object_model.get_articulation(name).mimic is None for name in joint_names),
        details="Expected a 2 x 3 array with one non-mimicked revolute hinge per flap.",
    )

    for row in range(2):
        for col in range(3):
            flap = object_model.get_part(f"flap_{row}_{col}")
            joint = object_model.get_articulation(f"frame_to_flap_{row}_{col}")
            pin_name = f"hinge_pin_{row}_{col}"

            ctx.allow_overlap(
                flap,
                frame,
                elem_a="barrel",
                elem_b=pin_name,
                reason=(
                    "The steel hinge pin is intentionally captured inside the "
                    "flap barrel so the flap is physically supported while it rotates."
                ),
            )
            ctx.expect_overlap(
                flap,
                frame,
                axes="x",
                elem_a="barrel",
                elem_b=pin_name,
                min_overlap=0.18,
                name=f"flap_{row}_{col} barrel surrounds its hinge pin",
            )

            rest_face = ctx.part_element_world_aabb(flap, elem="face")
            with ctx.pose({joint: 1.10}):
                open_face = ctx.part_element_world_aabb(flap, elem="face")

            ctx.check(
                f"flap_{row}_{col} swings outward on its own hinge",
                rest_face is not None
                and open_face is not None
                and open_face[0][1] < rest_face[0][1] - 0.10
                and open_face[0][2] > rest_face[0][2] + 0.03,
                details=f"rest_face={rest_face}, open_face={open_face}",
            )

    return ctx.report()


object_model = build_object_model()
