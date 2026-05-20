from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _bolt(part, *, x, y, z, radius, height, material, name):
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def _slot(part, *, x, y, z, length, width, material, name):
    _box(part, (length, width, 0.0018), (x, y, z), material, name)


def _add_bolt_pair(part, *, x, y_abs, z, radius, height, head_mat, slot_mat, name):
    for side, y in (("neg", -y_abs), ("pos", y_abs)):
        bolt_name = f"{name}_{side}"
        _bolt(
            part,
            x=x,
            y=y,
            z=z,
            radius=radius,
            height=height,
            material=head_mat,
            name=bolt_name,
        )
        _slot(
            part,
            x=x,
            y=y,
            z=z + 0.5 * height + 0.0003,
            length=radius * 1.55,
            width=radius * 0.36,
            material=slot_mat,
            name=f"{bolt_name}_slot",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_linear_extension")

    anodized = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    blued = model.material("blued_steel", rgba=(0.16, 0.18, 0.22, 1.0))
    ground = model.material("ground_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    carriage_mat = model.material("carriage_black", rgba=(0.025, 0.027, 0.030, 1.0))
    bolt_mat = model.material("black_oxide", rgba=(0.006, 0.006, 0.005, 1.0))
    rubber = model.material("urethane_stop", rgba=(0.02, 0.018, 0.012, 1.0))
    brass = model.material("brass_wiper", rgba=(0.85, 0.63, 0.28, 1.0))

    outer = model.part("outer_beam")
    _box(outer, (1.420, 0.250, 0.024), (0.710, 0.000, 0.012), anodized, "base_plate")
    _box(outer, (1.420, 0.024, 0.102), (0.710, -0.113, 0.074), anodized, "side_wall_0")
    _box(outer, (1.420, 0.024, 0.102), (0.710, 0.113, 0.074), anodized, "side_wall_1")
    _box(outer, (1.360, 0.040, 0.018), (0.710, -0.084, 0.124), anodized, "top_lip_0")
    _box(outer, (1.360, 0.040, 0.018), (0.710, 0.084, 0.124), anodized, "top_lip_1")
    _box(outer, (1.220, 0.012, 0.020), (0.700, -0.042, 0.032), ground, "floor_way_0")
    _box(outer, (1.220, 0.012, 0.020), (0.700, 0.042, 0.032), ground, "floor_way_1")
    _box(outer, (1.230, 0.007, 0.034), (0.700, -0.098, 0.074), ground, "side_way_0")
    _box(outer, (1.230, 0.007, 0.034), (0.700, 0.098, 0.074), ground, "side_way_1")
    _box(outer, (0.034, 0.092, 0.050), (0.034, 0.000, 0.049), anodized, "rear_stop_block")
    _box(outer, (0.034, 0.092, 0.050), (1.386, 0.000, 0.049), anodized, "front_stop_block")
    _box(outer, (0.026, 0.070, 0.028), (0.052, 0.000, 0.062), rubber, "rear_buffer")
    _box(outer, (0.026, 0.070, 0.028), (1.368, 0.000, 0.062), rubber, "front_buffer")
    _box(outer, (0.060, 0.330, 0.014), (0.160, 0.000, 0.007), anodized, "anchor_foot_0")
    _box(outer, (0.060, 0.330, 0.014), (1.260, 0.000, 0.007), anodized, "anchor_foot_1")
    for i, x in enumerate((0.120, 0.320, 0.520, 0.720, 0.920, 1.120, 1.300)):
        _add_bolt_pair(
            outer,
            x=x,
            y_abs=0.113,
            z=0.128,
            radius=0.0085,
            height=0.007,
            head_mat=bolt_mat,
            slot_mat=ground,
            name=f"outer_bolt_{i}",
        )

    stage_0 = model.part("rail_stage_0")
    _box(stage_0, (0.900, 0.094, 0.020), (0.450, 0.000, 0.000), blued, "stage_web")
    _box(stage_0, (0.900, 0.011, 0.052), (0.450, -0.051, 0.016), blued, "stage_side_0")
    _box(stage_0, (0.900, 0.011, 0.052), (0.450, 0.051, 0.016), blued, "stage_side_1")
    _box(stage_0, (0.025, 0.106, 0.044), (0.012, 0.000, 0.013), blued, "rear_end_cap")
    _box(stage_0, (0.025, 0.106, 0.044), (0.888, 0.000, 0.013), blued, "front_end_cap")
    _box(stage_0, (0.790, 0.010, 0.008), (0.455, -0.026, 0.014), ground, "upper_way_0")
    _box(stage_0, (0.790, 0.010, 0.008), (0.455, 0.026, 0.014), ground, "upper_way_1")
    _box(stage_0, (0.780, 0.005, 0.032), (0.455, -0.057, 0.018), ground, "outer_gib_0")
    _box(stage_0, (0.780, 0.005, 0.032), (0.455, 0.057, 0.018), ground, "outer_gib_1")
    _box(stage_0, (0.780, 0.012, 0.022), (0.455, -0.042, -0.021), ground, "lower_bearing_0")
    _box(stage_0, (0.780, 0.012, 0.022), (0.455, 0.042, -0.021), ground, "lower_bearing_1")
    _box(stage_0, (0.020, 0.034, 0.022), (0.045, -0.030, 0.052), rubber, "stage_buffer_0")
    _box(stage_0, (0.020, 0.034, 0.022), (0.045, 0.030, 0.052), rubber, "stage_buffer_1")
    _box(stage_0, (0.020, 0.034, 0.022), (0.855, -0.030, 0.052), rubber, "stage_buffer_2")
    _box(stage_0, (0.020, 0.034, 0.022), (0.855, 0.030, 0.052), rubber, "stage_buffer_3")
    for i, x in enumerate((0.140, 0.360, 0.580, 0.800)):
        _add_bolt_pair(
            stage_0,
            x=x,
            y_abs=0.050,
            z=0.044,
            radius=0.0062,
            height=0.006,
            head_mat=bolt_mat,
            slot_mat=ground,
            name=f"stage0_bolt_{i}",
        )

    stage_1 = model.part("rail_stage_1")
    _box(stage_1, (0.720, 0.074, 0.018), (0.360, 0.000, 0.000), blued, "inner_web")
    _box(stage_1, (0.720, 0.010, 0.043), (0.360, -0.041, 0.013), blued, "inner_side_0")
    _box(stage_1, (0.720, 0.010, 0.043), (0.360, 0.041, 0.013), blued, "inner_side_1")
    _box(stage_1, (0.022, 0.086, 0.036), (0.011, 0.000, 0.010), blued, "inner_rear_cap")
    _box(stage_1, (0.022, 0.086, 0.036), (0.709, 0.000, 0.010), blued, "inner_front_cap")
    _box(stage_1, (0.700, 0.009, 0.008), (0.360, -0.020, 0.0125), ground, "inner_way_0")
    _box(stage_1, (0.700, 0.009, 0.008), (0.360, 0.020, 0.0125), ground, "inner_way_1")
    _box(stage_1, (0.570, 0.006, 0.018), (0.410, -0.047, 0.018), ground, "inner_gib_0")
    _box(stage_1, (0.570, 0.006, 0.018), (0.410, 0.047, 0.018), ground, "inner_gib_1")
    _box(stage_1, (0.620, 0.009, 0.049), (0.370, -0.026, -0.0335), ground, "lower_guide_0")
    _box(stage_1, (0.620, 0.009, 0.049), (0.370, 0.026, -0.0335), ground, "lower_guide_1")
    _box(stage_1, (0.018, 0.026, 0.018), (0.047, -0.032, 0.043), rubber, "inner_buffer_0")
    _box(stage_1, (0.018, 0.026, 0.018), (0.047, 0.032, 0.043), rubber, "inner_buffer_1")
    _box(stage_1, (0.018, 0.026, 0.018), (0.673, -0.032, 0.043), rubber, "inner_buffer_2")
    _box(stage_1, (0.018, 0.026, 0.018), (0.673, 0.032, 0.043), rubber, "inner_buffer_3")
    for i, x in enumerate((0.120, 0.300, 0.480, 0.640)):
        _add_bolt_pair(
            stage_1,
            x=x,
            y_abs=0.039,
            z=0.037,
            radius=0.0052,
            height=0.0055,
            head_mat=bolt_mat,
            slot_mat=ground,
            name=f"stage1_bolt_{i}",
        )

    shoe = model.part("carriage_shoe")
    _box(shoe, (0.180, 0.090, 0.030), (0.090, 0.000, 0.000), carriage_mat, "shoe_body")
    _box(shoe, (0.158, 0.064, 0.018), (0.090, 0.000, 0.025), carriage_mat, "raised_pad")
    _box(shoe, (0.160, 0.012, 0.030), (0.090, -0.050, -0.005), carriage_mat, "clamp_ear_0")
    _box(shoe, (0.160, 0.012, 0.030), (0.090, 0.050, -0.005), carriage_mat, "clamp_ear_1")
    _box(shoe, (0.014, 0.106, 0.046), (0.176, 0.000, 0.006), carriage_mat, "nose_plate")
    _box(shoe, (0.130, 0.006, 0.012), (0.095, -0.056, 0.013), brass, "wiper_strip_0")
    _box(shoe, (0.130, 0.006, 0.012), (0.095, 0.056, 0.013), brass, "wiper_strip_1")
    _box(shoe, (0.130, 0.009, 0.0385), (0.080, -0.020, -0.03425), ground, "shoe_bearing_0")
    _box(shoe, (0.130, 0.009, 0.0385), (0.080, 0.020, -0.03425), ground, "shoe_bearing_1")
    for i, (x, y) in enumerate(((0.050, -0.025), (0.130, -0.025), (0.050, 0.025), (0.130, 0.025))):
        _bolt(
            shoe,
            x=x,
            y=y,
            z=0.036,
            radius=0.006,
            height=0.006,
            material=bolt_mat,
            name=f"shoe_bolt_{i}",
        )
        _slot(
            shoe,
            x=x,
            y=y,
            z=0.0394,
            length=0.010,
            width=0.0025,
            material=ground,
            name=f"shoe_bolt_{i}_slot",
        )

    model.articulation(
        "outer_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=stage_0,
        origin=Origin(xyz=(0.120, 0.000, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.22, lower=0.0, upper=0.340),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.080, 0.000, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.20, lower=0.0, upper=0.280),
    )
    model.articulation(
        "stage_1_to_shoe",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=shoe,
        origin=Origin(xyz=(0.500, 0.000, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_beam")
    stage_0 = object_model.get_part("rail_stage_0")
    stage_1 = object_model.get_part("rail_stage_1")
    shoe = object_model.get_part("carriage_shoe")
    j0 = object_model.get_articulation("outer_to_stage_0")
    j1 = object_model.get_articulation("stage_0_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_shoe")

    ctx.check(
        "three serial prismatic joints",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in (j0, j1, j2)),
        details="The extension should be built only from serial prismatic slide joints.",
    )
    ctx.check(
        "common slide axis",
        all(tuple(round(v, 6) for v in j.axis) == (1.0, 0.0, 0.0) for j in (j0, j1, j2)),
        details=f"axes={[j.axis for j in (j0, j1, j2)]}",
    )

    ctx.expect_within(
        stage_0,
        outer,
        axes="y",
        margin=0.0,
        name="first stage stays inside outer guide width",
    )
    ctx.expect_within(
        stage_1,
        stage_0,
        axes="y",
        margin=0.0,
        name="second stage stays inside first guide width",
    )
    ctx.expect_overlap(
        stage_0,
        outer,
        axes="x",
        min_overlap=0.80,
        name="collapsed first stage has retained insertion",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="x",
        min_overlap=0.60,
        name="collapsed second stage has retained insertion",
    )
    ctx.expect_overlap(
        shoe,
        stage_1,
        axes="x",
        min_overlap=0.16,
        name="collapsed shoe remains on terminal rail",
    )
    ctx.expect_contact(
        stage_0,
        outer,
        elem_a="lower_bearing_0",
        elem_b="floor_way_0",
        contact_tol=0.00001,
        name="first stage rides on outer precision way",
    )
    ctx.expect_contact(
        stage_1,
        stage_0,
        elem_a="lower_guide_0",
        elem_b="upper_way_0",
        contact_tol=0.00001,
        name="second stage rides on first-stage way",
    )
    ctx.expect_contact(
        shoe,
        stage_1,
        elem_a="shoe_bearing_0",
        elem_b="inner_way_0",
        contact_tol=0.00001,
        name="carriage shoe rides on terminal way",
    )

    rest_stage_0 = ctx.part_world_position(stage_0)
    rest_stage_1 = ctx.part_world_position(stage_1)
    rest_shoe = ctx.part_world_position(shoe)
    with ctx.pose({j0: 0.340, j1: 0.280, j2: 0.120}):
        ctx.expect_within(
            stage_0,
            outer,
            axes="y",
            margin=0.0,
            name="extended first stage stays laterally captured",
        )
        ctx.expect_within(
            stage_1,
            stage_0,
            axes="y",
            margin=0.0,
            name="extended second stage stays laterally captured",
        )
        ctx.expect_overlap(
            stage_0,
            outer,
            axes="x",
            min_overlap=0.78,
            name="extended first stage remains inserted",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="x",
            min_overlap=0.45,
            name="extended second stage remains inserted",
        )
        ctx.expect_overlap(
            shoe,
            stage_1,
            axes="x",
            min_overlap=0.08,
            name="extended shoe remains on terminal rail",
        )
        ctx.expect_contact(
            stage_0,
            outer,
            elem_a="lower_bearing_0",
            elem_b="floor_way_0",
            contact_tol=0.00001,
            name="extended first stage remains on outer way",
        )
        ctx.expect_contact(
            stage_1,
            stage_0,
            elem_a="lower_guide_0",
            elem_b="upper_way_0",
            contact_tol=0.00001,
            name="extended second stage remains on first way",
        )
        ctx.expect_contact(
            shoe,
            stage_1,
            elem_a="shoe_bearing_0",
            elem_b="inner_way_0",
            contact_tol=0.00001,
            name="extended shoe remains on terminal way",
        )
        end_stage_0 = ctx.part_world_position(stage_0)
        end_stage_1 = ctx.part_world_position(stage_1)
        end_shoe = ctx.part_world_position(shoe)

    ctx.check(
        "extension moves outward in sequence",
        rest_stage_0 is not None
        and rest_stage_1 is not None
        and rest_shoe is not None
        and end_stage_0 is not None
        and end_stage_1 is not None
        and end_shoe is not None
        and end_stage_0[0] > rest_stage_0[0] + 0.33
        and end_stage_1[0] > rest_stage_1[0] + 0.61
        and end_shoe[0] > rest_shoe[0] + 0.73,
        details=f"rest={(rest_stage_0, rest_stage_1, rest_shoe)}, extended={(end_stage_0, end_stage_1, end_shoe)}",
    )

    return ctx.report()


object_model = build_object_model()
