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
    model = ArticulatedObject(name="portable_synth_case")

    shell = model.material("charcoal_hard_shell", rgba=(0.05, 0.055, 0.06, 1.0))
    lid_shell = model.material("slate_lid_shell", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    bed_black = model.material("synth_panel_black", rgba=(0.015, 0.017, 0.020, 1.0))
    white_plastic = model.material("warm_white_keys", rgba=(0.90, 0.88, 0.82, 1.0))
    black_plastic = model.material("matte_black_keys", rgba=(0.005, 0.005, 0.006, 1.0))
    pointer_white = model.material("painted_pointer", rgba=(0.95, 0.95, 0.90, 1.0))

    base_x = 0.82
    base_y = 0.38
    wall_t = 0.025
    bottom_t = 0.025
    wall_h = 0.075
    top_z = bottom_t + wall_h
    gasket_h = 0.006
    hinge_x = -0.420
    hinge_z = top_z + gasket_h

    base = model.part("base")
    base.visual(
        Box((base_x, base_y, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=shell,
        name="floor_pan",
    )
    base.visual(
        Box((wall_t, base_y, wall_h)),
        origin=Origin(xyz=(base_x / 2.0 - wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=shell,
        name="front_wall",
    )
    base.visual(
        Box((wall_t, base_y, wall_h)),
        origin=Origin(xyz=(-base_x / 2.0 + wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=shell,
        name="rear_wall",
    )
    base.visual(
        Box((base_x - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, base_y / 2.0 - wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=shell,
        name="side_wall_0",
    )
    base.visual(
        Box((base_x - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -base_y / 2.0 + wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=shell,
        name="side_wall_1",
    )
    base.visual(
        Box((0.020, base_y - 0.050, gasket_h)),
        origin=Origin(xyz=(base_x / 2.0 - wall_t / 2.0, 0.0, top_z + gasket_h / 2.0)),
        material=rubber,
        name="front_gasket",
    )
    base.visual(
        Box((0.020, base_y - 0.050, gasket_h)),
        origin=Origin(xyz=(-base_x / 2.0 + wall_t + 0.010, 0.0, top_z + gasket_h / 2.0)),
        material=rubber,
        name="rear_gasket",
    )
    base.visual(
        Box((base_x - 0.090, 0.018, gasket_h)),
        origin=Origin(xyz=(0.0, base_y / 2.0 - wall_t - 0.009, top_z + gasket_h / 2.0)),
        material=rubber,
        name="side_gasket_0",
    )
    base.visual(
        Box((base_x - 0.090, 0.018, gasket_h)),
        origin=Origin(xyz=(0.0, -base_y / 2.0 + wall_t + 0.009, top_z + gasket_h / 2.0)),
        material=rubber,
        name="side_gasket_1",
    )

    # Small metal corner caps and a shallow molded front grip make the lower shell
    # read as a rugged portable case rather than a plain box.
    for ix, x in enumerate((-0.392, 0.392)):
        for iy, y in enumerate((-0.172, 0.172)):
            base.visual(
                Box((0.030, 0.030, 0.018)),
                origin=Origin(xyz=(x, y, 0.091)),
                material=steel,
                name=f"corner_cap_{ix}_{iy}",
            )
    base.visual(
        Box((0.020, 0.140, 0.024)),
        origin=Origin(xyz=(0.415, 0.0, 0.050)),
        material=rubber,
        name="front_grip",
    )

    hinge_r = 0.007
    hinge_y_centers = (-0.120, 0.120)
    for i, y in enumerate(hinge_y_centers):
        base.visual(
            Box((0.020, 0.090, 0.006)),
            origin=Origin(xyz=(-0.414, y, top_z + 0.003)),
            material=steel,
            name=f"rear_hinge_leaf_{i}",
        )
        base.visual(
            Cylinder(radius=hinge_r, length=0.080),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"rear_hinge_knuckle_{i}",
        )

    # Clevis-like front catch supports, two ears per catch.  The moving catch
    # barrels sit between these ears without needing a broad overlap allowance.
    catch_y_positions = (-0.110, 0.110)
    for i, y in enumerate(catch_y_positions):
        for side, yy in enumerate((y - 0.027, y + 0.027)):
            base.visual(
                Box((0.018, 0.008, 0.035)),
                origin=Origin(xyz=(0.418, yy, 0.070)),
                material=steel,
                name=f"catch_support_{i}_{side}",
            )

    bed = model.part("instrument_bed")
    bed.visual(
        Box((0.680, 0.280, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=bed_black,
        name="bed_panel",
    )
    for ix, x in enumerate((-0.270, 0.270)):
        for iy, y in enumerate((-0.110, 0.110)):
            bed.visual(
                Cylinder(radius=0.009, length=0.023),
                origin=Origin(xyz=(x, y, bottom_t + 0.0115)),
                material=steel,
                name=f"bed_standoff_{ix}_{iy}",
            )

    model.articulation(
        "base_to_bed",
        ArticulationType.FIXED,
        parent=base,
        child=bed,
        origin=Origin(),
    )

    # Piano-style alternating hinge barrels: the base owns the two outer
    # knuckles, the broad lid owns the central knuckle and leaf.
    lid = model.part("lid")
    lid.visual(
        Box((0.815, 0.360, 0.025)),
        origin=Origin(xyz=(0.4075, 0.0, 0.0525)),
        material=lid_shell,
        name="lid_top",
    )
    lid.visual(
        Box((0.025, 0.360, 0.040)),
        origin=Origin(xyz=(0.8275, 0.0, 0.020)),
        material=lid_shell,
        name="front_lip",
    )
    lid.visual(
        Box((0.840, 0.025, 0.040)),
        origin=Origin(xyz=(0.420, 0.180, 0.020)),
        material=lid_shell,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.840, 0.025, 0.040)),
        origin=Origin(xyz=(0.420, -0.180, 0.020)),
        material=lid_shell,
        name="side_lip_1",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.035, 0.130, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.008)),
        material=steel,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.010, 0.130, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, 0.025)),
        material=lid_shell,
        name="hinge_bridge",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    for i, y in enumerate(catch_y_positions):
        catch = model.part(f"front_catch_{i}")
        catch.visual(
            Cylinder(radius=0.006, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_barrel",
        )
        catch.visual(
            Box((0.012, 0.038, 0.080)),
            origin=Origin(xyz=(0.0, 0.0, 0.040)),
            material=steel,
            name="catch_arm",
        )
        catch.visual(
            Box((0.035, 0.042, 0.012)),
            origin=Origin(xyz=(-0.005, 0.0, 0.083)),
            material=steel,
            name="hook_tip",
        )
        model.articulation(
            f"catch_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=base,
            child=catch,
            origin=Origin(xyz=(0.432, y, 0.070)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.35),
        )

    # The instrument bed includes separate articulated synth controls so the
    # case still reads as a synth case when the lid is raised.
    white_count = 8
    white_w = 0.026
    white_gap = 0.002
    start_y = -((white_count * white_w + (white_count - 1) * white_gap) / 2.0) + white_w / 2.0
    for i in range(white_count):
        y = start_y + i * (white_w + white_gap)
        key = model.part(f"white_key_{i}")
        key.visual(
            Box((0.180, white_w, 0.009)),
            origin=Origin(xyz=(0.090, 0.0, 0.0045)),
            material=white_plastic,
            name="white_key",
        )
        model.articulation(
            f"white_key_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=bed,
            child=key,
            origin=Origin(xyz=(-0.085, y, 0.064)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0, lower=0.0, upper=0.045),
        )

    black_key_indices = (0, 1, 3, 4, 5)
    for j, i in enumerate(black_key_indices):
        y = start_y + (i + 0.5) * (white_w + white_gap)
        key = model.part(f"black_key_{j}")
        key.visual(
            Box((0.115, 0.016, 0.011)),
            origin=Origin(xyz=(0.0575, 0.0, 0.0165)),
            material=black_plastic,
            name="black_key",
        )
        key.visual(
            Box((0.050, 0.0015, 0.011)),
            origin=Origin(xyz=(0.030, 0.0, 0.0055)),
            material=black_plastic,
            name="black_key_stem",
        )
        model.articulation(
            f"black_key_{j}_hinge",
            ArticulationType.REVOLUTE,
            parent=bed,
            child=key,
            origin=Origin(xyz=(-0.085, y, 0.064)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0, lower=0.0, upper=0.040),
        )

    for i, x in enumerate((-0.250, -0.205, -0.160)):
        knob = model.part(f"control_knob_{i}")
        knob.visual(
            Cylinder(radius=0.013, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=black_plastic,
            name="knob_cap",
        )
        knob.visual(
            Box((0.004, 0.018, 0.002)),
            origin=Origin(xyz=(0.0, 0.004, 0.017)),
            material=pointer_white,
            name="pointer_mark",
        )
        model.articulation(
            f"control_knob_{i}_axis",
            ArticulationType.CONTINUOUS,
            parent=bed,
            child=knob,
            origin=Origin(xyz=(x, 0.095, 0.064)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bed = object_model.get_part("instrument_bed")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="front_lip",
        negative_elem="front_gasket",
        max_gap=0.002,
        max_penetration=0.0002,
        name="closed lid seats on front gasket",
    )
    ctx.expect_overlap(
        lid,
        bed,
        axes="xy",
        elem_a="lid_top",
        elem_b="bed_panel",
        min_overlap=0.20,
        name="broad lid spans the instrument bed",
    )
    ctx.expect_within(
        bed,
        base,
        axes="xy",
        inner_elem="bed_panel",
        outer_elem="floor_pan",
        margin=0.0,
        name="instrument bed is contained by the lower shell",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    with ctx.pose({lid_hinge: 1.15}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    ctx.check(
        "lid rotates upward around rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for i in range(2):
        catch = object_model.get_part(f"front_catch_{i}")
        pivot = object_model.get_articulation(f"catch_{i}_pivot")
        ctx.expect_overlap(
            catch,
            lid,
            axes="xy",
            elem_a="hook_tip",
            elem_b="front_lip",
            min_overlap=0.010,
            name=f"front catch {i} reaches over lid lip",
        )
        ctx.expect_gap(
            catch,
            lid,
            axis="z",
            positive_elem="hook_tip",
            negative_elem="front_lip",
            min_gap=0.0,
            max_gap=0.003,
            name=f"front catch {i} sits just above the lid lip",
        )
        rest_hook = ctx.part_element_world_aabb(catch, elem="hook_tip")
        with ctx.pose({pivot: 1.10}):
            rotated_hook = ctx.part_element_world_aabb(catch, elem="hook_tip")
        ctx.check(
            f"front catch {i} rotates outward on its support pivot",
            rest_hook is not None
            and rotated_hook is not None
            and rotated_hook[0][0] > rest_hook[0][0] + 0.035
            and rotated_hook[0][2] < rest_hook[0][2] - 0.004,
            details=f"rest={rest_hook}, rotated={rotated_hook}",
        )

    return ctx.report()


object_model = build_object_model()
